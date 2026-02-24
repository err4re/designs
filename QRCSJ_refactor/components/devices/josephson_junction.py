### phidl imports
from phidl import Device, Layer, LayerSet
from phidl.device_layout import DeviceReference
from phidl import quickplot as qp

import phidl.geometry as pg
import phidl.utilities as pu
import phidl.routing as pr
import phidl.path as pp

import gdspy

### general python imports
import numpy as np

from typing import Optional
from dataclasses import dataclass

### default layerset
from ..layerset.defaults import DEFAULT_DESIGN_LAYER_SET
default_design_ls = DEFAULT_DESIGN_LAYER_SET
default_ls = DEFAULT_DESIGN_LAYER_SET.layer_set


### define Dolan junction parameters
@dataclass(frozen=True)
class DolanJunctionParams:

    # overlap between structures for increased robustness of ebeam design
    overlap: float = 0.1

    junction_total_length: float = 10

    ### parameters for dolan bridge ###
    ###################################
    dolan_total_length: float = 4

    dolan_bridge_length: float = 0.12
    dolan_bridge_width: float = 0.4


    ### parameters undercuts ###
    ############################
    undercut_shape: str = 'H'

    # offset/spacing between undercut and main structures
    undercut_vertical_offset: float = 0.075
    undercut_horizontal_offset: float = 0.1

    # rectangular undercut
    rect_undercut_width: float = 1

    # H undercut
    h_undercut_width: float = 1
    h_undercut_height: float = 0.5


    ### parameters stress relief ###
    ################################
    stress_rect_width: float = 0.08
    stress_rect_length: float = 1.2

    # offset from bridge center
    stress_rect_offset: float = 1

    # set to 0 for no undercut
    stress_rect_undercut_width: float = 0.2
    stress_rect_undercut_length: float = 0.3


    stress_line_length: float = 1.2

    # offset from bridge center
    stress_line_offset: float = 1

    # set to 0 for no undercut
    stress_line_undercut_width: float = 0.2
    stress_line_undercut_length: float = 0.3


    ### Legacy params
    bridge_width: float = 0.4
    jj_width: float = 0.2
    jj_length: float = 4

    

    arm_width: float = 0.22
    total_length: float = 100

    undercut_width: float = 0.5

    extended_undercut: bool = False
    undercut_extension: float = 0.5
    undercut_spacing_v: float = 0.075
    undercut_spacing_h: float = 0.1
    undercut_slit: float = 0.04

    stress_boxes_width: float = 0.09
    stress_boxes_height: float = 0.6
    stress_boxes_undercut_height: float = 0.6
    stress_boxes_undercut_width: float = 0.4

    undercut_shape: str = 'H_asym_slits'

    layer: Layer = default_ls['ebeam']
    junction_layer: Layer = default_ls['ebeam_junctions']
    junction_line_layer: Layer = default_ls['ebeam_junctions']

    stress_layer: Layer = default_ls['ebeam_junctions_stress']
    stress_line_layer: Layer = default_ls['ebeam_junctions_stress']

    undercut_layer: Layer = default_ls['jj_undercut']

    def __post_init__(self):

        # validate layers


        # validate undercut shape
        valid_shapes = ['rectangle', 'H', 'H_rounded', 'H_slit', 'H_boxes', 'H_sym_slits', 'H_asym_slits', 'H_gills','none']
        if self.undercut_shape not in valid_shapes:
            raise ValueError(f"Unsupported undercut shape: {self.undercut_shape}")


### define Dolan Josephson Junction class
class DolanJunction:
    """
    Class to represent and generate a Dolan Josephson Junction centered around the origin.

    Attributes:
        device (Device): A PHIDL device object representing the junction.
        jj_params (JJParams): Parameters used for junction generation.
        undercut (DeviceReference): A PHIDL device reference object representing the undercut.
        junction (DeviceReference): A PHIDL device reference object representing the arms of the junction.
    """

    def __init__(self) -> None:
        self.jj_params: Optional[DolanJunctionParams] = None

        ### phidl device containing the full junction
        self.device: Optional[Device] = None


        ### phidl references for all elements making up the full junction

        # both sides of dolan bridge
        self.dolan_1_ref:Optional[DeviceReference] = None
        self.dolan_2_ref: Optional[DeviceReference] = None

        # dolan bridge undercut
        self.undercut_ref: Optional[DeviceReference] = None

        # arms connecting to dolan junction on both sides
        self.arm_1_ref: Optional[DeviceReference] = None
        self.arm_2_ref: Optional[DeviceReference] = None

        # stress relief structures around dolan bridge
        self.stress_1_ref: Optional[DeviceReference] = None
        self.stress_2_ref: Optional[DeviceReference] = None


    @staticmethod
    def build_dolan_rect(dolan_junction_params: DolanJunctionParams) -> Device:

        Dolan = Device('dolan_junction')

        # compute length for single dolan rectangle, add overlap for overlap with junction arm
        length = (dolan_junction_params.dolan_total_length - dolan_junction_params.dolan_bridge_width)/2 + dolan_junction_params.overlap

        Compass = pg.compass(size=(dolan_junction_params.dolan_bridge_length, length), layer=dolan_junction_params.junction_layer)
        
        Dolan << Compass
        [Dolan.add_port(port=port) for port in Compass.ports]

        return Dolan
    
    @staticmethod
    def build_dolan_line(dolan_junction_params: DolanJunctionParams) -> Device:

        Dolan = Device('dolan_line_junction')

        # compute length for single dolan rectangle, add overlap for overlap with junction arm
        length = (dolan_junction_params.dolan_total_length - dolan_junction_params.dolan_bridge_width)/2 + dolan_junction_params.overlap

        # create zero width path using gdspy, not possible with phidl
        # line width is set by dose in junction_line_layer
        fp = gdspy.FlexPath([(0, 0), (0, length)],
                             width=0,  
                             layer=dolan_junction_params.junction_line_layer.gds_layer,
                             datatype=dolan_junction_params.junction_line_layer.gds_datatype,
                             ends='flush',
                             gdsii_path=True)
        
        Dolan << fp
        # add compass style ports
        Dolan.add_port(name = "N", midpoint=(0, length), orientation=90)
        Dolan.add_port(name = "S", midpoint=(0, 0), orientation=-90)

        return Dolan
    
    @staticmethod
    def build_arm_rect(dolan_junction_params: DolanJunctionParams) -> Device:

        Arm = Device('dolan_arm')

        # compute length for single arm rectangle
        length = (dolan_junction_params.junction_total_length - dolan_junction_params.dolan_total_length)/2

        Compass = pg.compass(size=(dolan_junction_params.arm_width, length), layer=dolan_junction_params.junction_layer)
        
        Arm << Compass
        [Arm.add_port(port=port) for port in Compass.ports]

        return Arm
    

    # @staticmethod
    # def build_undercut(dolan_junction_params: DolanJunctionParams) -> Device:

        
    @staticmethod
    def build_undercut_rect(dolan_junction_params: DolanJunctionParams) -> Device:

        Undercut = Device('dolan_rect_undercut')

        # compute height for undercut rectangle to respect chosen undercut offsets
        height = dolan_junction_params.bridge_width - 2*dolan_junction_params.undercut_vertical_offset

        Compass = pg.compass(size=(dolan_junction_params.rect_undercut_width, height), layer=dolan_junction_params.undercut_layer)

        Undercut << Compass
        [Undercut.add_port(port=port) for port in Compass.ports]

        return Undercut
    
    @staticmethod
    def build_undercut_h(dolan_junction_params: DolanJunctionParams) -> Device:

        # H shaped undercut will be composed of three rectangles, center, left and right
        Undercut = Device('dolan_h_undercut')

        # central undercut rectangle, right under the dolan bridge
        # compute height for undercut rectangle to respect chosen undercut offsets
        height = dolan_junction_params.bridge_width - 2*dolan_junction_params.undercut_vertical_offset

        # compute width for central undercut rectangle in H undercut
        width = dolan_junction_params.arm_width + 2*dolan_junction_params.undercut_horizontal_offset

        Compass = pg.compass(size=(width, height), layer=dolan_junction_params.undercut_layer)

        compass_center_ref = Undercut << Compass
        [Undercut.add_port(port=port) for port in Compass.ports]

        # two side undercut rectangles, touching the sides of the central rectangle
        height_side = dolan_junction_params.h_undercut_height
        width_side = (dolan_junction_params.h_undercut_width - width)/2

        # left side undercut rectangle
        Compass = pg.compass(size=(width_side, height_side), layer=dolan_junction_params.undercut_layer)
        compass_left_ref = Undercut << Compass
        compass_left_ref.connect(port='E', destination=Undercut.ports['W'])

        # right side undercut rectrangle
        Compass = pg.compass(size=(width_side, height_side), layer=dolan_junction_params.undercut_layer)
        compass_right_ref = Undercut << Compass
        compass_right_ref.connect(port='W', destination=Undercut.ports['E'])        

        return Undercut
    

    # @staticmethod
    # def build_stress_relief(dolan_junction_params: DolanJunctionParams) -> Device:


    @staticmethod
    def build_stress_relief_rect(dolan_junction_params: DolanJunctionParams) -> Device:

        # stress relief made from two rectangles
        Stress_Relief = Device('stress_relief_rect')

        # left side
        Compass = pg.compass(size=(dolan_junction_params.stress_rect_width, dolan_junction_params.stress_rect_length), layer=dolan_junction_params.stress_layer)
        
        # optional undercuts at ends of rectangle
        make_undercuts = (dolan_junction_params.stress_rect_undercut_width > 0 and dolan_junction_params.stress_rect_undercut_length > 0)
        
        if make_undercuts:
            Undercut = pg.compass(size=(dolan_junction_params.stress_rect_undercut_width, dolan_junction_params.stress_rect_undercut_length), layer=dolan_junction_params.undercut_layer)
            undercut_top_ref = Compass << Undercut
            undercut_top_ref.connect(port=undercut_top_ref.ports['S'], destination=Compass.ports['N'])

            Undercut = pg.compass(size=(dolan_junction_params.stress_rect_undercut_width, dolan_junction_params.stress_rect_undercut_length), layer=dolan_junction_params.undercut_layer)
            undercut_bot_ref = Compass << Undercut
            undercut_bot_ref.connect(port=undercut_bot_ref.ports['N'], destination=Compass.ports['S'])

        # add to device and move to correct position on the left
        compass_left_ref = Stress_Relief << Compass

        y = (dolan_junction_params.stress_rect_offset + dolan_junction_params.stress_rect_width)/2
        compass_left_ref.move(origin=compass_left_ref.center, destination=(y, 0))

        # right side
        Compass = pg.compass(size=(dolan_junction_params.stress_rect_width, dolan_junction_params.stress_rect_length), layer=dolan_junction_params.stress_layer)
        
        # optional undercuts at ends of rectangle
        if make_undercuts:
            Undercut = pg.compass(size=(dolan_junction_params.stress_rect_undercut_width, dolan_junction_params.stress_rect_undercut_length), layer=dolan_junction_params.undercut_layer)
            undercut_top_ref = Compass << Undercut
            undercut_top_ref.connect(port=undercut_top_ref.ports['S'], destination=Compass.ports['N'])

            Undercut = pg.compass(size=(dolan_junction_params.stress_rect_undercut_width, dolan_junction_params.stress_rect_undercut_length), layer=dolan_junction_params.undercut_layer)
            undercut_bot_ref = Compass << Undercut
            undercut_bot_ref.connect(port=undercut_bot_ref.ports['N'], destination=Compass.ports['S'])

        # add to device and move to correct position on the left
        compass_right_ref = Stress_Relief << Compass

        y = - (dolan_junction_params.stress_rect_offset + dolan_junction_params.stress_rect_width)/2
        compass_right_ref.move(origin=compass_left_ref.center, destination=(y, 0))

        return Stress_Relief




    @staticmethod
    def build_stress_relief_line(dolan_junction_params: DolanJunctionParams) -> Device:

        Stress_Relief = Device('stress_relief_line')

        # left side
        # line width is set by dose in stress_line_layer
        fp = gdspy.FlexPath([(0, 0), (0, dolan_junction_params.stress_line_length)],
                             width=0,  
                             layer=dolan_junction_params.stress_line_layer.gds_layer,
                             datatype=dolan_junction_params.stress_line_layer.gds_datatype,
                             ends='flush',
                             gdsii_path=True)
        
        Compass = Device('Compass')
        Compass << fp
        Compass.add_port(name = "N", midpoint=(0, dolan_junction_params.stress_line_length), orientation=90)
        Compass.add_port(name = "S", midpoint=(0, 0), orientation=-90)
        
        # optional undercuts at ends of rectangle
        make_undercuts = (dolan_junction_params.stress_rect_undercut_width > 0 and dolan_junction_params.stress_rect_undercut_length > 0)
        
        if make_undercuts:
            Undercut = pg.compass(size=(dolan_junction_params.stress_rect_undercut_width, dolan_junction_params.stress_rect_undercut_length), layer=dolan_junction_params.undercut_layer)
            undercut_top_ref = Compass << Undercut
            undercut_top_ref.connect(port=undercut_top_ref.ports['S'], destination=Compass.ports['N'])

            Undercut = pg.compass(size=(dolan_junction_params.stress_rect_undercut_width, dolan_junction_params.stress_rect_undercut_length), layer=dolan_junction_params.undercut_layer)
            undercut_bot_ref = Compass << Undercut
            undercut_bot_ref.connect(port=undercut_bot_ref.ports['N'], destination=Compass.ports['S'])

        # add to device and move to correct position on the left
        compass_left_ref = Stress_Relief << Compass

        y = (dolan_junction_params.stress_rect_offset + dolan_junction_params.stress_rect_width)/2
        compass_left_ref.move(origin=compass_left_ref.center, destination=(y, 0))

        # right side
        # line width is set by dose in stress_line_layer
        fp = gdspy.FlexPath([(0, 0), (0, dolan_junction_params.stress_line_length)],
                             width=0,  
                             layer=dolan_junction_params.stress_line_layer.gds_layer,
                             datatype=dolan_junction_params.stress_line_layer.gds_datatype,
                             ends='flush',
                             gdsii_path=True)
        
        Compass = Device('Compass')
        Compass << fp
        Compass.add_port(name = "N", midpoint=(0, dolan_junction_params.stress_line_length), orientation=90)
        Compass.add_port(name = "S", midpoint=(0, 0), orientation=-90)

        # optional undercuts at ends of rectangle
        if make_undercuts:
            Undercut = pg.compass(size=(dolan_junction_params.stress_rect_undercut_width, dolan_junction_params.stress_rect_undercut_length), layer=dolan_junction_params.undercut_layer)
            undercut_top_ref = Compass << Undercut
            undercut_top_ref.connect(port=undercut_top_ref.ports['S'], destination=Compass.ports['N'])

            Undercut = pg.compass(size=(dolan_junction_params.stress_rect_undercut_width, dolan_junction_params.stress_rect_undercut_length), layer=dolan_junction_params.undercut_layer)
            undercut_bot_ref = Compass << Undercut
            undercut_bot_ref.connect(port=undercut_bot_ref.ports['N'], destination=Compass.ports['S'])

        # add to device and move to correct position on the left
        compass_right_ref = Stress_Relief << Compass

        y = - (dolan_junction_params.stress_rect_offset + dolan_junction_params.stress_rect_width)/2
        compass_right_ref.move(origin=compass_left_ref.center, destination=(y, 0))

        return Stress_Relief

    # @staticmethod
    # def build_stress_relief_asym(dolan_junction_params:DolanJunctionParams) -> Device:



