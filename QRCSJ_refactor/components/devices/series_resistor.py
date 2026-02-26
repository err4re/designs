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
from functools import cached_property

### default layerset
from ..layerset.defaults import DEFAULT_DESIGN_LAYER_SET

default_design_ls = DEFAULT_DESIGN_LAYER_SET
default_ls = default_design_ls.layer_set


### define series resistor parameters
@dataclass(frozen=True)
class SeriesResistorParams:

    overlap: float = 0.1

    # total height of resistor without connectors, controls spacing between resistive line segments
    resistor_total_height: float = 40

    # sets which side the top connector will be facing towards (other components)
    # bot connector faces same side for even number of resistive line segments
    # bot connector faces opposite side for odd number of resistive line segments
    facing: str = 'left'

    ### resistive line segments ###
    ###############################

    # width of resistive line segments and thickness of Cr film set resistance per square
    segment_width: float = 0.11
    segment_length: float = 25

    # number of segments connected in series, can be even or odd
    segment_number: int = 10

    # minimal spacing between resistive line segments
    segment_minimal_spacing: float = 1


    ### elbow (connecting resistive line segements) ###
    ###################################################

    elbow_width: float = 0.8

    # vertical margin to ensure first Al evaporation covers both resistive line segments connected to elbow
    elbow_margin: float = 0.8
    # margin on top, bottom or both sides of elbow
    elbow_margin_side: str = 'both'


    ### connectors (at ends of resistor) ###
    ########################################

    connectors: bool = True

    connector_height: float = 1.2
    connector_length: float = 2

    connector_margin: float = 0.8

    ### undercut ###
    ################

    undercut_offset: float = 0.08
    undercut_width: float = 0.8

    ### Layers ###
    ##############

    segment_layer: Layer = default_ls['ebeam_strong']
    elbow_layer: Layer = default_ls['ebeam']
    connector_layer: Layer = default_ls['ebeam_low']

    elbow_undercut_layer: Layer = default_ls['undercut']
    connector_undercut_layer: Layer = default_ls['undercut']


    def __post_init__(self):

        # validate facing/orientation
        valid_facing = ['left', 'right']
        if self.facing not in valid_facing:
            raise ValueError(f'Unsupported orientation, facing: {self.facing}! Resistor can only face: {valid_facing}.')
        
        # validate elbow margin side
        valid_elbow_margin_side = ['both', 'top', 'bot']
        if self.elbow_margin_side not in valid_elbow_margin_side:
            raise ValueError(f'Unsupported elbow margin side: {self.elbow_margin_side}! Valid options: {valid_elbow_margin_side}.')

        # validate spacing between resistive line segments
        if self.segment_spacing < self.segment_minimal_spacing:
            raise ValueError(f'Spacing between resistive line segments is smaller than minimal spacing: {self.segment_spacing} < {self.segment_minimal_spacing}! Increase resistors total height or reduce number of segments.')




    @cached_property
    def segment_spacing(self) -> float:
        """
        This cached property computes the vertical spacing between resistive segments.
        """

        total_spacing = self.resistor_total_height - self.segment_number*self.segment_width
        spacing = total_spacing/(self.segment_number - 1)

        return spacing
    

    @cached_property
    def segment_number_even(self) -> bool:
        """
        This cached property checks if the number of resistive line segments is even.
        """

        segment_number_even = (self.segment_number+1)%2

        return segment_number_even



### define series resistor class
class SeriesResistor:

    def __init__(self) -> None:
        
        ### phidl device containing the full series resistor
        self.device: Optional[Device] = None

        # parameters used to build resistor
        self.params: Optional[SeriesResistorParams] = None

        ### all elements making up full series resistor

        # all resistive line segments are references to the same device
        self.Segment: Optional[Device] = None
        self.segment_refs: Optional[list[DeviceReference]] = None

        # all right elbows are references to the same device
        self.Elbow_Right: Optional[Device] = None
        self.Elbow_Right_Undercut: Optional[Device] = None

        self.elbow_right_refs: Optional[list[DeviceReference]] = None
        self.elbow_right_undercut_refs: Optional[list[DeviceReference]] = None

        # all left elbows are references to the same device
        self.Elbow_Left: Optional[Device] = None
        self.Elbow_Left_Undercut: Optional[Device] = None

        self.elbow_left_refs: Optional[list[DeviceReference]] = None
        self.elbow_left_undercut_refs: Optional[list[DeviceReference]] = None

        # top connector (other circuit elements will connect to the resistor here)
        self.Connector_Top: Optional[Device] = None
        self.connector_top_ref: Optional[Device] = None

        # bot connector (other circuit elements will connect to the resistor here)
        self.Connector_Bot: Optional[Device] = None
        self.connector_bot_ref: Optional[DeviceReference] = None



    def build_series_resistor(self, res_params: SeriesResistorParams) -> Device:

        if self.device is not None:
            raise RuntimeWarning('This series resistor has already been built and can by accessed through self.device!')
        
        Series_Resistor = Device('series_resistor')

    @staticmethod
    def _build_segment(res_params: SeriesResistorParams) -> Device:

        Segment = Device('segment_resistive')

        Segment << pg.rectangle(size=(res_params.segment_length + 2*res_params.overlap, res_params.segment_width), layer=res_params.segment_layer)

        Segment.add_port(name='left', midpoint=[res_params.overlap, res_params.segment_width/2], width=res_params.segment_width)
        Segment.add_port(name='right', midpoint=[res_params.segment_length + res_params.overlap, res_params.segment_width/2], width=res_params.segment_width)

        return Segment
    
    @staticmethod
    def _build_connector_top(res_params: SeriesResistorParams) -> Device:

        Connector_Top = Device('connector_top')

        connector_height = res_params.connector_height + res_params.connector_margin
        Connector_Top << pg.rectangle(size=(res_params.connector_length, connector_height), layer=res_params.connector_layer)

        if res_params.facing == 'left':
            Connector_Top.add_port(name='left',
                                   midpoint=[0, connector_height/2],
                                   width=connector_height,
                                   orientation=180)
            
            Connector_Top.add_port(name='right',
                                   midpoint=[res_params.connector_length, res_params.connector_margin + res_params.segment_width/2],
                                   width=res_params.segment_width,
                                   orientation=0)
            
        elif res_params.facing == 'right':
            Connector_Top.add_port(name='left',
                                   midpoint=[0, res_params.connector_margin + res_params.segment_width/2],
                                   width=res_params.segment_width,
                                   orientation=180)
            
            Connector_Top.add_port(name='right',
                                   midpoint=[res_params.connector_length, connector_height/2],
                                   width=connector_height,
                                   orientation=0)
            

        return Connector_Top
    
    

    def _build_connector_top_udnercut(self, res_params) -> Device:

    def _build_connector_bot_udnercut(self, res_params) -> Device:


    @staticmethod
    def _build_connector_bot(res_params: SeriesResistorParams) -> Device:

        Connector_Bot = Device('connector_bot')

        connector_height = res_params.connector_height + res_params.connector_margin
        Connector_Bot << pg.rectangle(size=(res_params.connector_length, connector_height), layer=res_params.connector_layer)

        # check if bot connector should face the same side as top connector
        if res_params.segment_number_even:
            facing = res_params.facing
        else:
            if res_params.facing == 'left':
                facing = 'right'
            elif res_params.facing == 'right':
                facing = 'left'

        if facing == 'left':
            Connector_Bot.add_port(name='left',
                                   midpoint=[0, connector_height/2],
                                   width=connector_height,
                                   orientation=180)
            
            Connector_Bot.add_port(name='right',
                                   midpoint=[res_params.connector_length, res_params.connector_height - res_params.segment_width/2],
                                   width=res_params.segment_width,
                                   orientation=0)
            
        if facing == 'right':
            Connector_Bot.add_port(name='left',
                                   midpoint=[0, res_params.connector_height - res_params.segment_width/2],
                                   width=res_params.segment_width,
                                   orientation=180)
            
            Connector_Bot.add_port(name='right',
                                   midpoint=[res_params.connector_length, connector_height/2],
                                   width=connector_height,
                                   orientation=0)
            

        return Connector_Bot   

    @staticmethod
    def _build_elbow_right(res_params: SeriesResistorParams) -> Device:

        Elbow_Right = Device('elbow_right')

        # elbow height without margin
        elbow_height = res_params.segment_spacing + 2*res_params.segment_width

        margin_side = res_params.elbow_margin_side
        # height of elbow and position of ports depends on margin side: top, bot or both
        if margin_side == 'bot':
            elbow_height += res_params.elbow_margin
            Elbow_Right << pg.rectangle(size=(res_params.elbow_width, elbow_height), layer=res_params.elbow_layer)

            Elbow_Right.add_port(name='top',
                                 midpoint=(0, elbow_height - res_params.segment_width/2),
                                 width=res_params.segment_width,
                                 orientation=180)
            Elbow_Right.add_port(name='bot',
                                 midpoint=(0, res_params.segment_width/2 + res_params.elbow_margin),
                                 width=res_params.segment_width,
                                 orientation=180)

            return Elbow_Right
        
        if margin_side == 'top':
            elbow_height += res_params.elbow_margin
            Elbow_Right << pg.rectangle(size=(res_params.elbow_width, elbow_height), layer=res_params.elbow_layer)

            Elbow_Right.add_port(name='top',
                                 midpoint=(0, elbow_height - res_params.elbow_margin - res_params.segment_width/2),
                                 width=res_params.segment_width,
                                 orientation=180)
            Elbow_Right.add_port(name='bot',
                                 midpoint=(0, res_params.segment_width/2),
                                 width=res_params.segment_width,
                                 orientation=180)

            return Elbow_Right
        
        if margin_side == 'both':
            elbow_height += 2*res_params.elbow_margin
            Elbow_Right << pg.rectangle(size=(res_params.elbow_width, elbow_height), layer=res_params.elbow_layer)

            Elbow_Right.add_port(name='top',
                                 midpoint=(0, elbow_height - res_params.elbow_margin - res_params.segment_width/2),
                                 width=res_params.segment_width,
                                 orientation=180)
            Elbow_Right.add_port(name='bot',
                                 midpoint=(0, res_params.elbow_margin + res_params.segment_width/2),
                                 width=res_params.segment_width,
                                 orientation=180)

            return Elbow_Right
        
        raise NotImplementedError(
            f"Elbow margin side '{margin_side}' is valid but no builder method is implemented for it yet."
        )
    
    @staticmethod
    def _build_elbow_left(res_params: SeriesResistorParams) -> Device:

        Elbow_Left = Device('elbow_left')

        # elbow height without margin
        elbow_height = res_params.segment_spacing + 2*res_params.segment_width

        margin_side = res_params.elbow_margin_side
        # height of elbow and position of ports depends on margin side: top, bot or both
        if margin_side == 'bot':
            elbow_height += res_params.elbow_margin
            Elbow_Left << pg.rectangle(size=(res_params.elbow_width, elbow_height), layer=res_params.elbow_layer)

            Elbow_Left.add_port(name='top',
                                 midpoint=(res_params.elbow_width, elbow_height - res_params.segment_width/2),
                                 width=res_params.segment_width,
                                 orientation=0)
            Elbow_Left.add_port(name='bot',
                                 midpoint=(res_params.elbow_width, res_params.segment_width/2 + res_params.elbow_margin),
                                 width=res_params.segment_width,
                                 orientation=0)

            return Elbow_Left
        
        if margin_side == 'top':
            elbow_height += res_params.elbow_margin
            Elbow_Left << pg.rectangle(size=(res_params.elbow_width, elbow_height), layer=res_params.elbow_layer)

            Elbow_Left.add_port(name='top',
                                 midpoint=(res_params.elbow_width, elbow_height - res_params.elbow_margin - res_params.segment_width/2),
                                 width=res_params.segment_width,
                                 orientation=0)
            Elbow_Left.add_port(name='bot',
                                 midpoint=(res_params.elbow_width, res_params.segment_width/2),
                                 width=res_params.segment_width,
                                 orientation=0)

            return Elbow_Left
        
        if margin_side == 'both':
            elbow_height += 2*res_params.elbow_margin
            Elbow_Left << pg.rectangle(size=(res_params.elbow_width, elbow_height), layer=res_params.elbow_layer)

            Elbow_Left.add_port(name='top',
                                 midpoint=(res_params.elbow_width, elbow_height - res_params.elbow_margin - res_params.segment_width/2),
                                 width=res_params.segment_width,
                                 orientation=0)
            Elbow_Left.add_port(name='bot',
                                 midpoint=(res_params.elbow_width, res_params.elbow_margin + res_params.segment_width/2),
                                 width=res_params.segment_width,
                                 orientation=0)

            return Elbow_Left
        
        raise Not