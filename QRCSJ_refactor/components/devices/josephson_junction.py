### phidl imports
from phidl import Device, Layer, LayerSet
from phidl.device_layout import DeviceReference, Polygon
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
default_ls = default_design_ls.layer_set


### define Dolan junction parameters
@dataclass(frozen=True)
class DolanJunctionParams:

    # overlap between structures for increased robustness of ebeam design
    overlap: float = 0.1

    # sets total length of junction taking into account desired overlap with higher level geometry, e.g. SQUID
    junction_total_length: float = 10

    ### dolan bridge parameters ###
    ###################################

    # sets length of dolan bridge part
    dolan_total_length: float = 4

    dolan_bridge_length: float = 0.12
    dolan_bridge_width: float = 0.4


    ### undercut parameters ###
    ###########################
    undercut_shape: str = 'H'

    # offset/spacing between undercut and main structures
    undercut_vertical_offset: float = 0.075
    undercut_horizontal_offset: float = 0.1

    # rectangular undercut
    rect_undercut_width: float = 1

    # H undercut
    h_undercut_width: float = 0.85
    h_undercut_height: float = 0.5

    ### arn parameters ###
    ######################
    arm_width = 0.22


    ### stress relief parameters ###
    ################################
    stress_shape: str = 'rectangle'

    stress_rect_width: float = 0.08
    stress_rect_length: float = 1.2

    # offset from bridge center
    stress_rect_offset: float = 1.2

    # set to 0 for no undercut
    stress_rect_undercut_width: float = 0.2
    stress_rect_undercut_length: float = 0.3


    stress_line_length: float = 1.2

    # offset from bridge center
    stress_line_offset: float = 1

    # set to 0 for no undercut
    stress_line_undercut_width: float = 0.2
    stress_line_undercut_length: float = 0.3


    stress_asym_poly_width: float = 0.09
    stress_asym_poly_length: float = 1

    # spacing between two polys on the same side
    stress_asym_poly_spacing: float = 0.17

    # offset/spacing from dolan arm
    stress_asym_poly_offset: float = 0.22


    stress_asym_line_length: float = 1

    # spacing between two polys on the same side
    stress_asym_line_spacing: float = 0.17

    # offset/spacing from dolan arm
    stress_asym_line_offset: float = 0.22



    ### Layers ###
    ##############
    arm_layer: Layer = default_ls['ebeam']
    junction_layer: Layer = default_ls['ebeam_junctions']
    junction_line_layer: Layer = default_ls['ebeam_junctions']

    stress_layer: Layer = default_ls['ebeam_junctions_stress']
    stress_line_layer: Layer = default_ls['ebeam_junctions_stress']

    undercut_layer: Layer = default_ls['jj_undercut']



    def __post_init__(self):

        # validate lengths
        if self.dolan_total_length > self.junction_total_length:
            raise ValueError(f'Dolan geometry with length: {self.dolan_total_length} cannot be longer than complete junction with length: {self.junction_total_length}!')

        # validate layers, check for example that they are of DesignLayerType ebeam exposure


        # validate undercut shape
        valid_undercut_shapes = ['rectangle', 'H', 'none']
        if self.undercut_shape not in valid_undercut_shapes:
            raise ValueError(f"Unsupported undercut shape: {self.undercut_shape}!")
        

        # validate stress shape
        valid_stress_shapes = ['rectangle', 'line', 'asym_poly', 'asym_line', 'none']
        if self.stress_shape not in valid_stress_shapes:
            raise ValueError(f"Unsupported stress relief shape: {self.stress_shape}!")



### define Dolan Josephson Junction class
class DolanJunction:
    """
    Class to represent and generate a Dolan Josephson Junction centered around the origin.

    Attributes:
        device (Device): A PHIDL device object representing the junction.
        params (DolanJunctionParams): Parameters used for junction generation.
    """

    def __init__(self) -> None:

        ### phidl device containing the full junction
        self.device: Optional[Device] = None

        # parameters used to build the full device
        self.params: Optional[DolanJunctionParams] = None


        ### phidl references for all elements making up the full junction

        # both sides of dolan bridge
        self.Dolan_1: Optional[Device] = None
        self.Dolan_2: Optional[Device] = None

        self.dolan_1_ref: Optional[DeviceReference] = None
        self.dolan_2_ref: Optional[DeviceReference] = None

        # dolan bridge undercut
        self.Undercut: Optional[Device] = None

        self.undercut_ref: Optional[DeviceReference] = None

        # arms connecting to dolan junction on both sides
        self.Arm_1: Optional[Device] = None
        self.Arm_2: Optional[Device] = None

        self.arm_1_ref: Optional[DeviceReference] = None
        self.arm_2_ref: Optional[DeviceReference] = None

        # stress relief structures around dolan bridge
        self.Stress_1: Optional[Device] = None
        self.Stress_2: Optional[Device] = None

        self.stress_1_ref: Optional[DeviceReference] = None
        self.stress_2_ref: Optional[DeviceReference] = None


    def build_dolan_junction(self, dolan_junction_params: DolanJunctionParams) -> Device:

        if self.device is not None:
            raise RuntimeWarning('This Dolan junction has already been built and can by accessed through self.device!')
        
        Dolan_Junction = Device('dolan_junction')

        self.Undercut = self.build_undercut(dolan_junction_params)

        self.undercut_ref = Dolan_Junction << self.Undercut

        self.Dolan_1 = DolanJunction.build_dolan_rect(dolan_junction_params)
        self.Dolan_2 = DolanJunction.build_dolan_rect(dolan_junction_params)

        self.dolan_1_ref = Dolan_Junction << self.Dolan_1
        self.dolan_2_ref = Dolan_Junction << self.Dolan_2

        # connect dolan structures to undercut to form bridge, undercut is centered around zero
        self.dolan_1_ref.connect(port=self.dolan_1_ref.ports['S'], destination=self.undercut_ref.ports['in'])
        self.dolan_2_ref.connect(port=self.dolan_2_ref.ports['N'], destination=self.undercut_ref.ports['out'])

        self.Arm_1 = DolanJunction.build_arm_rect(dolan_junction_params)
        self.Arm_2 = DolanJunction.build_arm_rect(dolan_junction_params)

        self.arm_1_ref = Dolan_Junction << self.Arm_1
        self.arm_2_ref = Dolan_Junction << self.Arm_2

        # connect arms to dolan bridge part of the junction
        self.arm_1_ref.connect(port=self.arm_1_ref.ports['S'], destination=self.dolan_1_ref.ports['in'])
        self.arm_2_ref.connect(port=self.arm_2_ref.ports['N'], destination=self.dolan_2_ref.ports['out'])

        Dolan_Junction.add_port(name='in', port=self.arm_1_ref.ports['in'])
        Dolan_Junction.add_port(name='out', port=self.arm_2_ref.ports['out'])

        # build stress relief structures
        self.Stress_1, self.Stress_2 = self.build_stress_relief(dolan_junction_params)

        # add stress relief to junction, they are in the correct position given that junction is centered around zero
        self.stress_1_ref = Dolan_Junction << self.Stress_1
        self.stress_2_ref = Dolan_Junction << self.Stress_2

        # save device and parameters that were used to generate it
        self.device = Dolan_Junction
        self.params = dolan_junction_params

        return Dolan_Junction



    @staticmethod
    def build_dolan_rect(dolan_junction_params: DolanJunctionParams) -> Device:

        Dolan = Device('dolan_junction_bridge')

        # compute length for single dolan rectangle, add overlap for overlap with junction arm
        length = (dolan_junction_params.dolan_total_length - dolan_junction_params.dolan_bridge_width)/2 + dolan_junction_params.overlap

        Compass = pg.compass(size=(dolan_junction_params.dolan_bridge_length, length), layer=dolan_junction_params.junction_layer)
        
        Dolan << Compass
        [Dolan.add_port(port=port) for port in Compass.ports.values()]

        # add ports to connect to arm design
        Dolan.add_port(name='in',
                          midpoint=Dolan.ports['N'].midpoint - [0, dolan_junction_params.overlap],
                          orientation=Dolan.ports['N'].orientation)
        
        Dolan.add_port(name='out',
                          midpoint=Dolan.ports['S'].midpoint + [0, dolan_junction_params.overlap],
                          orientation=Dolan.ports['S'].orientation)
        

        return Dolan
    
    @staticmethod
    def build_dolan_line(dolan_junction_params: DolanJunctionParams) -> Device:

        Dolan = Device('dolan_line_junction_bridge')

        # compute length for single dolan rectangle, add overlap for overlap with junction arm
        length = (dolan_junction_params.dolan_total_length - dolan_junction_params.dolan_bridge_width)/2 + dolan_junction_params.overlap

        # create zero width path using gdspy later, not possible with phidl
        # two point polygon to be replaced with gdspy zero width path only at the end of design procedure, to ensure phidl transformation work (e.g. mirror)
        # line width is set by dose in junction_line_layer
        Dolan.add_polygon(points = [(0, 0), (0, length)], layer = dolan_junction_params.junction_line_layer)
   
        # add compass style ports
        Dolan.add_port(name = "N", midpoint=(0, length), orientation=90)
        Dolan.add_port(name = "S", midpoint=(0, 0), orientation=-90)

        return Dolan
    
    @staticmethod
    def build_arm_rect(dolan_junction_params: DolanJunctionParams) -> Device:

        Arm = Device('dolan_arm')

        # compute length for single arm rectangle
        length = (dolan_junction_params.junction_total_length - dolan_junction_params.dolan_total_length)/2 + dolan_junction_params.overlap

        Compass = pg.compass(size=(dolan_junction_params.arm_width, length), layer=dolan_junction_params.arm_layer)
        
        Arm << Compass
        [Arm.add_port(port=port) for port in Compass.ports.values()]

        # add ports to connect to squid design
        Arm.add_port(name='in',
                          midpoint=Arm.ports['N'].midpoint - [0, dolan_junction_params.overlap],
                          orientation=Arm.ports['N'].orientation)
        
        Arm.add_port(name='out',
                          midpoint=Arm.ports['S'].midpoint + [0, dolan_junction_params.overlap],
                          orientation=Arm.ports['S'].orientation)


        return Arm
    

    def build_undercut(self, dolan_junction_params: DolanJunctionParams) -> Device:
        shape = dolan_junction_params.undercut_shape

        if shape == 'none':
            return Device('dolan_no_undercut')
        if shape == 'rectangle':
            return DolanJunction.build_undercut_rect(dolan_junction_params)
        if shape == 'H':
            return DolanJunction.build_undercut_h(dolan_junction_params)

        raise NotImplementedError(
            f"Undercut shape '{shape}' is valid but no builder method is implemented for it yet!"
        )
        
    @staticmethod
    def build_undercut_rect(dolan_junction_params: DolanJunctionParams) -> Device:

        Undercut = Device('dolan_rect_undercut')

        # compute height for undercut rectangle to respect chosen undercut offsets
        height = dolan_junction_params.dolan_bridge_width - 2*dolan_junction_params.undercut_vertical_offset

        Compass = pg.compass(size=(dolan_junction_params.rect_undercut_width, height), layer=dolan_junction_params.undercut_layer)

        Undercut << Compass
        [Undercut.add_port(port=port) for port in Compass.ports.values()]

        # add ports to connect to dolan design
        Undercut.add_port(name='in',
                          midpoint=Undercut.ports['N'].midpoint + [0, dolan_junction_params.undercut_vertical_offset],
                          orientation=Undercut.ports['N'].orientation)
        
        Undercut.add_port(name='out',
                          midpoint=Undercut.ports['S'].midpoint - [0, dolan_junction_params.undercut_vertical_offset],
                          orientation=Undercut.ports['S'].orientation)

        return Undercut
    
    @staticmethod
    def build_undercut_h(dolan_junction_params: DolanJunctionParams) -> Device:

        # H shaped undercut will be composed of three rectangles, center, left and right
        Undercut = Device('dolan_h_undercut')

        # central undercut rectangle, right under the dolan bridge
        # compute height for undercut rectangle to respect chosen undercut offsets
        height = dolan_junction_params.dolan_bridge_width - 2*dolan_junction_params.undercut_vertical_offset

        # compute width for central undercut rectangle in H undercut
        width = dolan_junction_params.dolan_bridge_length + 2*dolan_junction_params.undercut_horizontal_offset

        Compass = pg.compass(size=(width, height), layer=dolan_junction_params.undercut_layer)

        compass_center_ref = Undercut << Compass
        [Undercut.add_port(port=port) for port in Compass.ports.values()]

        # add ports to connect to dolan design
        Undercut.add_port(name='in',
                          midpoint=Undercut.ports['N'].midpoint + [0, dolan_junction_params.undercut_vertical_offset],
                          orientation=Undercut.ports['N'].orientation)
        
        Undercut.add_port(name='out',
                          midpoint=Undercut.ports['S'].midpoint - [0, dolan_junction_params.undercut_vertical_offset],
                          orientation=Undercut.ports['S'].orientation)
        

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
    

    def build_stress_relief(self, dolan_junction_params: DolanJunctionParams) -> Device:
        shape = dolan_junction_params.stress_shape

        if shape == 'none':
            return Device('stress_relief_none')
        if shape == 'rectangle':
            return DolanJunction.build_stress_relief_rect(dolan_junction_params)
        if shape == 'line':
            return DolanJunction.build_stress_relief_line(dolan_junction_params)
        if shape == 'asym_poly':
            return DolanJunction.build_stress_relief_asym_poly(dolan_junction_params)
        if shape == 'asym_line':
            return DolanJunction.build_stress_relief_asym_line(dolan_junction_params)

        raise NotImplementedError(
            f"Stress relief shape '{shape}' is valid but no builder method is implemented for it yet."
        )

    @staticmethod
    def _build_stress_relief_single_rect(dolan_junction_params: DolanJunctionParams) -> Device:

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

        return Compass


    @staticmethod
    def build_stress_relief_rect(dolan_junction_params: DolanJunctionParams) -> tuple[Device, Device]:

        # stress relief first rectangle
        Stress_Relief_1 = Device('stress_relief_rect_1')

        # left side
        Compass = DolanJunction._build_stress_relief_single_rect(dolan_junction_params)

        # add to device and move to correct position on the left
        compass_left_ref = Stress_Relief_1 << Compass

        x = (dolan_junction_params.stress_rect_offset + dolan_junction_params.stress_rect_width)/2
        compass_left_ref.move(origin=compass_left_ref.center, destination=(x, 0))

        # stress relief seccond rectangle (other side)
        Stress_Relief_2 = Device('stress_relief_rect_2')

        # right side
        Compass = DolanJunction._build_stress_relief_single_rect(dolan_junction_params)

        # add to device and move to correct position on the right
        compass_right_ref = Stress_Relief_2 << Compass

        x = - (dolan_junction_params.stress_rect_offset + dolan_junction_params.stress_rect_width)/2
        compass_right_ref.move(origin=compass_right_ref.center, destination=(x, 0))

        return Stress_Relief_1, Stress_Relief_2
    
    @staticmethod
    def _build_stress_relief_single_line(dolan_junction_params: DolanJunctionParams) -> Device:
        
        Compass = Device('Compass')

        # create zero width path using gdspy later, not possible with phidl
        # two point polygon to be replaced with gdspy zero width path only at the end of design procedure, to ensure phidl transformation work (e.g. mirror)
        # line width is set by dose in stress_line_layer
        Compass.add_polygon(points=[(0, 0), (0, dolan_junction_params.stress_line_length)], layer=dolan_junction_params.stress_line_layer.gds_layer)
        
        
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

        return Compass


    @staticmethod
    def build_stress_relief_line(dolan_junction_params: DolanJunctionParams) -> tuple[Device, Device]:

        Stress_Relief_1 = Device('stress_relief_line_1')

        # left side   
        Compass = DolanJunction._build_stress_relief_single_line(dolan_junction_params)
        
        # add to device and move to correct position on the left
        compass_left_ref = Stress_Relief_1 << Compass

        x = (dolan_junction_params.stress_line_offset)/2
        compass_left_ref.move(origin=compass_left_ref.center, destination=(x, 0))


        Stress_Relief_2 = Device('stress_relief_line_2')

        # right side
        # line width is set by dose in stress_line_layer
        Compass = DolanJunction._build_stress_relief_single_line(dolan_junction_params)

        # add to device and move to correct position on the left
        compass_right_ref = Stress_Relief_2 << Compass

        x = - (dolan_junction_params.stress_line_offset)/2
        compass_right_ref.move(origin=compass_right_ref.center, destination=(x, 0))

        return Stress_Relief_1, Stress_Relief_2
    
    @staticmethod
    def _build_stress_relief_asym_poly_side(dolan_junction_params: DolanJunctionParams) -> Device:

        Side = Device('stress_asym_poly_side')

        # polys angled at 45° and optimized for clean fracturing
        # long poly
        points = [(0,0),
                  (dolan_junction_params.stress_asym_poly_length/np.sqrt(2), dolan_junction_params.stress_asym_poly_length/np.sqrt(2)),
                  (dolan_junction_params.stress_asym_poly_length/np.sqrt(2) - dolan_junction_params.stress_asym_poly_width*np.sqrt(2), dolan_junction_params.stress_asym_poly_length/np.sqrt(2)),
                  (- dolan_junction_params.stress_asym_poly_width*np.sqrt(2),0)
                  ]
        
        Side.add_polygon(points, layer=dolan_junction_params.stress_layer)

        # short poly
        offset = (dolan_junction_params.stress_asym_poly_spacing + dolan_junction_params.stress_asym_poly_width)*np.sqrt(2)

        points = [(dolan_junction_params.stress_asym_poly_length/np.sqrt(2), dolan_junction_params.stress_asym_poly_length/np.sqrt(2) - offset),
                  (dolan_junction_params.stress_asym_poly_length/np.sqrt(2) - dolan_junction_params.stress_asym_poly_width*np.sqrt(2), dolan_junction_params.stress_asym_poly_length/np.sqrt(2) - offset),
                  (dolan_junction_params.stress_asym_poly_length/np.sqrt(2) - dolan_junction_params.stress_asym_poly_width*np.sqrt(2) + (dolan_junction_params.stress_asym_poly_length/np.sqrt(2) - offset), 0),
                  (dolan_junction_params.stress_asym_poly_length/np.sqrt(2) - dolan_junction_params.stress_asym_poly_width*np.sqrt(2) + (dolan_junction_params.stress_asym_poly_length/np.sqrt(2) - offset) + dolan_junction_params.stress_asym_poly_width*np.sqrt(2), 0)
                  ]
        
        Side.add_polygon(points, layer=dolan_junction_params.stress_layer)

        # orient vertically, i.e. pointing from bottom to top
        Side.rotate(90)

        return Side 

    @staticmethod
    def build_stress_relief_asym_poly(dolan_junction_params: DolanJunctionParams) -> tuple[Device, Device]:

        Stress_Relief_1 = Device('stress_relief_asym_poly_1')

        # left side
        Side = DolanJunction._build_stress_relief_asym_poly_side(dolan_junction_params)

        # add to device and move to correct position on the left
        side_left_ref = Stress_Relief_1 << Side

        x = - (dolan_junction_params.dolan_bridge_length/2 + dolan_junction_params.stress_asym_poly_offset)
        side_left_ref.move(origin=(side_left_ref.xmax, side_left_ref.y), destination=(x, 0))

        Stress_Relief_2 = Device('stress_relief_asym_poly_2')

        # right side
        Side = DolanJunction._build_stress_relief_asym_poly_side(dolan_junction_params).mirror()

        # add to device and move to correct position on the right
        side_left_ref = Stress_Relief_1 << Side

        x = (dolan_junction_params.dolan_bridge_length/2 + dolan_junction_params.stress_asym_poly_offset)
        side_left_ref.move(origin=(side_left_ref.xmin, side_left_ref.y), destination=(x, 0))



        return Stress_Relief_1, Stress_Relief_2


    @staticmethod
    def _build_stress_relief_asym_line_side(dolan_junction_params: DolanJunctionParams) -> Device:

        Side = Device('stress_asym_line_side')

        # polys angled at 45°

        # long line

        # create zero width path using gdspy later, not possible with phidl
        # two point polygon to be replaced with gdspy zero width path only at the end of design procedure, to ensure phidl transformation work (e.g. mirror)
        # line width is set by dose in stress_line_layer
        Side.add_polygon(points=[(0, 0), (dolan_junction_params.stress_asym_line_length/np.sqrt(2), dolan_junction_params.stress_asym_line_length/np.sqrt(2))],
                         layer=dolan_junction_params.stress_line_layer)

        # short poly
        offset = (dolan_junction_params.stress_asym_line_spacing)*np.sqrt(2)

        
        Side.add_polygon(points=[(dolan_junction_params.stress_asym_line_length/np.sqrt(2), dolan_junction_params.stress_asym_line_length/np.sqrt(2) - offset), (dolan_junction_params.stress_asym_line_length/np.sqrt(2)  + dolan_junction_params.stress_asym_line_length/np.sqrt(2) - offset, 0)],
                         layer=dolan_junction_params.stress_line_layer)

        # orient vertically, i.e. pointing from bottom to top
        Side.rotate(90)

        return Side 
        
    @staticmethod
    def build_stress_relief_asym_line(dolan_junction_params: DolanJunctionParams) -> tuple[Device, Device]:

        Stress_Relief_1 = Device('stress_relief_asym_line_1')

        # left side
        Side = DolanJunction._build_stress_relief_asym_line_side(dolan_junction_params)

        # add to device and move to correct position on the left
        side_left_ref = Stress_Relief_1 << Side

        x = - (dolan_junction_params.dolan_bridge_length/2 + dolan_junction_params.stress_asym_line_offset)
        side_left_ref.move(origin=(side_left_ref.xmax, side_left_ref.y), destination=(x, 0))

        Stress_Relief_2 = Device('stress_relief_asym_line_2')

        # right side
        Side = DolanJunction._build_stress_relief_asym_line_side(dolan_junction_params).mirror()

        # add to device and move to correct position on the right
        side_left_ref = Stress_Relief_1 << Side

        x = (dolan_junction_params.dolan_bridge_length/2 + dolan_junction_params.stress_asym_line_offset)
        side_left_ref.move(origin=(side_left_ref.xmin, side_left_ref.y), destination=(x, 0))



        return Stress_Relief_1, Stress_Relief_2


