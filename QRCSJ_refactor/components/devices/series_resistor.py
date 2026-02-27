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
    segment_line_layer: Layer = default_ls['ebeam_strong']
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
        self.segment_refs: Optional[list[DeviceReference]] = []

        # all right elbows are references to the same device
        self.Elbow_Right: Optional[Device] = None
        self.Elbow_Right_Undercut: Optional[Device] = None

        self.elbow_right_refs: Optional[list[DeviceReference]] = []
        self.elbow_right_undercut_ref: Optional[DeviceReference] = None

        # all left elbows are references to the same device
        self.Elbow_Left: Optional[Device] = None
        self.Elbow_Left_Undercut: Optional[Device] = None

        self.elbow_left_refs: Optional[list[DeviceReference]] = []
        self.elbow_left_undercut_ref: Optional[DeviceReference] = None

        # top connector (other circuit elements will connect to the resistor here)
        self.Connector_Top: Optional[Device] = None
        self.connector_top_ref: Optional[DeviceReference] = None

        self.Connector_Top_Undercut: Optional[Device] = None
        self.connector_top_undercut_ref: Optional[DeviceReference] = None

        # bot connector (other circuit elements will connect to the resistor here)
        self.Connector_Bot: Optional[Device] = None
        self.connector_bot_ref: Optional[DeviceReference] = None

        self.Connector_Bot_Undercut: Optional[Device] = None
        self.connector_bot_undercut_ref: Optional[DeviceReference] = None



    def build_series_resistor(self, res_params: SeriesResistorParams) -> Device:

        if self.device is not None:
            raise RuntimeWarning('This series resistor has already been built and can by accessed through self.device!')
        
        Series_Resistor = Device('series_resistor')

        # add all segments needed for this resistor
        # add option to choose betwenn rectangle or single pass line in the future?
        self.Segment = SeriesResistor._build_segment_rect(res_params)
        [self.segment_refs.append(Series_Resistor << self.Segment) for i in range(res_params.segment_number)]
        
        self.Elbow_Right = SeriesResistor._build_elbow_right(res_params)
        self.Elbow_Left = SeriesResistor._build_elbow_left(res_params)

        if res_params.connectors:
            self.Connector_Top = SeriesResistor._build_connector_top(res_params)
            self.connector_top_ref = Series_Resistor << self.Connector_Top

            self.Connector_Bot = SeriesResistor._build_connector_bot(res_params)
            self.connector_bot_ref = Series_Resistor << self.Connector_Bot

        # order of assembly/connecting elements together depends on which side the series resistor faces
        # facing left starts with right elbow
        if res_params.facing == 'left':

            # connect top connector to first resistive segment
            if res_params.connectors:
                self.connector_top_ref.connect(port=self.connector_top_ref.ports['right'], destination=self.segment_refs[0].ports['left'])

            # connect top to bottom segments and elbows between segments
            for i,segment_ref in enumerate(self.segment_refs[1:]):

                if (i+1)%2:
                    elbow_right_ref = Series_Resistor << self.Elbow_Right
                    elbow_right_ref.connect(port=elbow_right_ref.ports['top'], destination=self.segment_refs[i].ports['right'])

                    segment_ref.connect(port=segment_ref.ports['right'], destination=elbow_right_ref.ports['bot'])

                    self.elbow_right_refs.append(elbow_right_ref)
                else:
                    elbow_left_ref = Series_Resistor << self.Elbow_Left
                    elbow_left_ref.connect(port=elbow_left_ref.ports['top'], destination=self.segment_refs[i].ports['left'])

                    segment_ref.connect(port=segment_ref.ports['left'], destination=elbow_left_ref.ports['bot'])

                    self.elbow_left_refs.append(elbow_left_ref)

            # connect bot connector last
            if res_params.connectors:
                if res_params.segment_number_even:
                    self.connector_bot_ref.connect(port=self.connector_bot_ref.ports['right'], destination=self.segment_refs[-1].ports['left'])
                else:
                    self.connector_bot_ref.connect(port=self.connector_bot_ref.ports['left'], destination=self.segment_refs[-1].ports['right'])
                
        # facing right starts with left elbow
        elif res_params.facing == 'right':

            if res_params.connectors:
                self.segment_refs[0].connect(port=self.segment_refs[0].ports['right'], destination=self.connector_top_ref.ports['left'])

            for i,segment_ref in enumerate(self.segment_refs[1:]):

                if (i+1)%2:
                    elbow_left_ref = Series_Resistor << self.Elbow_Left
                    elbow_left_ref.connect(port=elbow_left_ref.ports['top'], destination=self.segment_refs[i].ports['left'])

                    segment_ref.connect(port=segment_ref.ports['left'], destination=elbow_left_ref.ports['bot'])

                    self.elbow_left_refs.append(elbow_left_ref)
                else:
                    elbow_right_ref = Series_Resistor << self.Elbow_Right
                    elbow_right_ref.connect(port=elbow_right_ref.ports['top'], destination=self.segment_refs[i].ports['right'])

                    segment_ref.connect(port=segment_ref.ports['right'], destination=elbow_right_ref.ports['bot'])

                    self.elbow_right_refs.append(elbow_right_ref)

            if res_params.connectors:
                if res_params.segment_number_even:
                    self.connector_bot_ref.connect(port=self.connector_bot_ref.ports['left'], destination=self.segment_refs[-1].ports['right'])
                else:
                    self.connector_bot_ref.connect(port=self.connector_bot_ref.ports['right'], destination=self.segment_refs[-1].ports['left'])


        ### when creating elbow undercuts make sure the elbows actually exist
        ### e.g. for two segments only one type of elbow will be created
        if self.elbow_right_refs:

            self.Elbow_Right_Undercut = self._build_elbow_right_undercut(Series_Resistor, res_params)
            self.Elbow_Right_Undercut.move(origin= self.Elbow_Right_Undercut.center,
                                               destination=self.Elbow_Right.center + [res_params.undercut_width/2, 0]
                                               )
         
            self.elbow_right_undercut_ref = self.Elbow_Right << self.Elbow_Right_Undercut


        if self.elbow_left_refs:

            self.Elbow_Left_Undercut = self._build_elbow_left_undercut(Series_Resistor, res_params)
            self.Elbow_Left_Undercut.move(origin= self.Elbow_Left_Undercut.center,
                                               destination=self.Elbow_Left.center + [res_params.undercut_width/2, 0]
                                               )
         
            self.elbow_right_undercut_ref = self.Elbow_Left << self.Elbow_Left_Undercut


        ### connector undercuts
        ### these undercut devices are in the same places as the corresponding references, this makes for easier modification later
        if res_params.connectors:
            self.Connector_Top_Undercut = self._build_connector_top_undercut(Series_Resistor, res_params)
            self.Connector_Top_Undercut.move(origin= self.Connector_Top_Undercut.center,
                                               destination=self.Connector_Top.center + [res_params.undercut_width/2, 0]
                                               )
         
            self.connector_top_undercut_ref = self.Connector_Top << self.Connector_Top_Undercut

            self.Connector_Bot_Undercut = self._build_connector_bot_undercut(Series_Resistor, res_params)
            self.Connector_Bot_Undercut.move(origin= self.Connector_Bot_Undercut.center,
                                               destination=self.Connector_Bot.center + [res_params.undercut_width/2, 0]
                                               )
         
            self.connector_bot_undercut_ref = self.Connector_Bot << self.Connector_Bot_Undercut




        # add ports to connect to other components
        self._add_ports(Series_Resistor, res_params)

        # save device and params
        self.device = Series_Resistor
        self.params = res_params

        return Series_Resistor


    def _add_ports(self, Series_Resistor: Device, res_params: SeriesResistorParams):
        """
        Adds ports to the Series_Resistor device for connection with other components.
        """

        ### left and right ports
        # if the series resistor has connectors add ports on the connectors
        if res_params.connectors:
            
            if self.connector_top_ref is None:
                raise RuntimeError(f'Top connector has not been built yet, self.connector_top_ref is {self.connector_top_ref}!')

            if self.connector_bot_ref is None:
                raise RuntimeError(f'Bot connector has not been built yet, self.connector_bot_ref is {self.connector_bot_ref}!')


            if res_params.facing == 'left':
                Series_Resistor.add_port(name='top_left', port=self.connector_top_ref.ports['left'])

                if res_params.segment_number_even:
                    Series_Resistor.add_port(name='bot_left', port=self.connector_bot_ref.ports['left'])
                else:
                    Series_Resistor.add_port(name='bot_right', port=self.connector_bot_ref.ports['right'])

            elif res_params.facing == 'right':
                Series_Resistor.add_port(port=self.connector_top_ref.ports['right'])

                if res_params.segment_number_even:
                    Series_Resistor.add_port(name='bot_right', port=self.connector_bot_ref.ports['right'])
                else:
                    Series_Resistor.add_port(name='bot_left', port=self.connector_bot_ref.ports['left'])
        
        # if the series resistor does not have connectors add ports on the first and last resistive segment
        else:
            if res_params.facing == 'left':
                Series_Resistor.add_port(name='top_left', port=self.segment_refs[0].ports['left'])

                if res_params.segment_number_even:
                    Series_Resistor.add_port(name='bot_left', port=self.segment_refs[-1].ports['left'])
                else:
                    Series_Resistor.add_port(name='bot_right', port=self.segment_refs[-1].ports['right'])

            elif res_params.facing == 'right':
                Series_Resistor.add_port(name='top_right', port=self.segment_refs[0].ports['right'])

                if res_params.segment_number_even:
                    Series_Resistor.add_port(name='bot_right', port=self.segment_refs[-1].ports['right'])
                else:
                    Series_Resistor.add_port(name='bot_left', port=self.segment_refs[-1].ports['left'])


        ### top and bot ports
        if res_params.connectors:
            
            Series_Resistor.add_port(name='top_top', port=self.connector_top_ref.ports['top'])

            Series_Resistor.add_port(name='bot_bot', port=self.connector_bot_ref.ports['bot'])



        


    @staticmethod
    def _build_segment_rect(res_params: SeriesResistorParams) -> Device:

        Segment = Device('segment_resistive_rect')

        Segment << pg.rectangle(size=(res_params.segment_length + 2*res_params.overlap, res_params.segment_width), layer=res_params.segment_layer)

        Segment.add_port(name='left',
                         midpoint=[res_params.overlap, res_params.segment_width/2],
                         width=res_params.segment_width,
                         orientation=180)
        Segment.add_port(name='right',
                         midpoint=[res_params.segment_length + res_params.overlap, res_params.segment_width/2],
                         width=res_params.segment_width,
                         orientation=0)

        return Segment
    
    @staticmethod
    def _build_segment_line(res_params: SeriesResistorParams) -> Device:

        Segment = Device('segment_resistive_line')

        Segment.add_polygon(points=[(0, 0), (res_params.segment_length + 2*res_params.overlap, 0)], layer=res_params.segment_line_layer)

        Segment.add_port(name='left',
                         midpoint=[res_params.overlap, 0],
                         width=res_params.segment_width,
                         orientation=180)
        Segment.add_port(name='right',
                         midpoint=[res_params.segment_length + res_params.overlap, 0],
                         width=res_params.segment_width,
                         orientation=0)

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
        
        x_center = (Connector_Top.xmax + Connector_Top.xmin)/2
        Connector_Top.add_port(name='top',
                               midpoint=(x_center, Connector_Top.ymax),
                               width=res_params.connector_length,
                               orientation=90)

        return Connector_Top
    
    

    def _build_connector_top_undercut(self, Resistor_Device: Device, res_params: SeriesResistorParams) -> Device:
        """
        Build undercut for top connector. Needs Connector_Top and will respect undercut offset with respect to all elements already built into the Resistor_Device.
        """

        if self.Connector_Top is None:
            raise RuntimeError(f'Top connector has not been built yet, self.Connector_Top is {self.Connector_Top}!')

        Connector_Top_Undercut = Device('connector_top_undercut')

        Undercut = Device()

        points = [(self.connector_top_ref.xmin, self.connector_top_ref.ymin - res_params.undercut_width),
                  (self.connector_top_ref.xmin, self.connector_top_ref.ymax + res_params.undercut_width),
                  (self.connector_top_ref.xmax + res_params.undercut_width, self.connector_top_ref.ymax + res_params.undercut_width),
                  (self.connector_top_ref.xmax+ res_params.undercut_width, self.connector_top_ref.ymin - res_params.undercut_width)]
        Undercut.add_polygon(points)

        Undercut_Offset = pg.offset(elements=[Resistor_Device], distance=res_params.undercut_offset)

        Undercut = pg.boolean(A=Undercut,
                              B=Undercut_Offset,
                              operation='A-B',
                              layer=res_params.connector_undercut_layer)
        
        Connector_Top_Undercut << Undercut

        return Connector_Top_Undercut


    def _build_connector_bot_undercut(self, Resistor_Device: Device, res_params: SeriesResistorParams) -> Device:
        """
        Build undercut for bot connector. Needs Connector_Bot and will respect undercut offset with respect to all elements already built into the Resistor_Device.
        """

        if self.Connector_Bot is None:
            raise RuntimeError(f'Top connector has not been built yet, self.Connector_Bot is {self.Connector_Bot}!')

        Connector_Bot_Undercut = Device('connector_bot_undercut')

        Undercut = Device()

        points = [(self.connector_bot_ref.xmin, self.connector_bot_ref.ymin - res_params.undercut_width),
                  (self.connector_bot_ref.xmin, self.connector_bot_ref.ymax + res_params.undercut_width),
                  (self.connector_bot_ref.xmax + res_params.undercut_width, self.connector_bot_ref.ymax + res_params.undercut_width),
                  (self.connector_bot_ref.xmax+ res_params.undercut_width, self.connector_bot_ref.ymin - res_params.undercut_width)]
        Undercut.add_polygon(points)

        Undercut_Offset = pg.offset(elements=[Resistor_Device], distance=res_params.undercut_offset)

        Undercut = pg.boolean(A=Undercut,
                              B=Undercut_Offset,
                              operation='A-B',
                              layer=res_params.connector_undercut_layer)
        
        Connector_Bot_Undercut << Undercut

        return Connector_Bot_Undercut



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
            
        x_center = (Connector_Bot.xmax + Connector_Bot.xmin)/2
        Connector_Bot.add_port(name='bot',
                               midpoint=(x_center, Connector_Bot.ymin),
                               width=res_params.connector_length,
                               orientation=-90)

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
        
        raise NotImplementedError(
            f"Elbow margin side '{margin_side}' is valid but no builder method is implemented for it yet."
        )


    def _build_elbow_right_undercut(self, Resistor_Device: Device, res_params: SeriesResistorParams) -> Device:
        """
        Build undercut for right elbow. Needs Elbow_Right and will respect undercut offset with respect to all elements already built into the Resistor_Device.
        """

        if self.Elbow_Right is None:
            raise RuntimeError(f'Right elbow whas not been built yet, self.Elbow_Right is {self.Elbow_Right}!')

        Elbow_Right_Undercut = Device('elbow_right_undercut')

        Undercut = Device()

        points = [(self.elbow_right_refs[0].xmin, self.elbow_right_refs[0].ymin - res_params.undercut_width),
                  (self.elbow_right_refs[0].xmin, self.elbow_right_refs[0].ymax + res_params.undercut_width),
                  (self.elbow_right_refs[0].xmax + res_params.undercut_width, self.elbow_right_refs[0].ymax + res_params.undercut_width),
                  (self.elbow_right_refs[0].xmax + res_params.undercut_width, self.elbow_right_refs[0].ymin - res_params.undercut_width)]
        Undercut.add_polygon(points)

        Undercut_Offset = pg.offset(elements=[Resistor_Device], distance=res_params.undercut_offset)

        Undercut = pg.boolean(A=Undercut,
                              B=Undercut_Offset,
                              operation='A-B',
                              layer=res_params.elbow_undercut_layer)
        
        Elbow_Right_Undercut << Undercut

        return Elbow_Right_Undercut


    def _build_elbow_left_undercut(self, Resistor_Device: Device, res_params: SeriesResistorParams) -> Device:
        """
        Build undercut for left elbow. Needs Elbow_Right and will respect undercut offset with respect to all elements already built into the Resistor_Device.
        """

        if self.Elbow_Left is None:
            raise RuntimeError(f'Left elbow whas not been built yet, self.Elbow_Left is {self.Elbow_Left}!')

        Elbow_Left_Undercut = Device('elbow_left_undercut')

        Undercut = Device()

        # points are actually the same as for right elbow undercut, opportunity for combining the two?
        points = [(self.elbow_left_refs[0].xmin, self.elbow_left_refs[0].ymin - res_params.undercut_width),
                  (self.elbow_left_refs[0].xmin, self.elbow_left_refs[0].ymax + res_params.undercut_width),
                  (self.elbow_left_refs[0].xmax + res_params.undercut_width, self.elbow_left_refs[0].ymax + res_params.undercut_width),
                  (self.elbow_left_refs[0].xmax+ res_params.undercut_width, self.elbow_left_refs[0].ymin - res_params.undercut_width)]
        Undercut.add_polygon(points)

        Undercut_Offset = pg.offset(elements=[Resistor_Device], distance=res_params.undercut_offset)

        Undercut = pg.boolean(A=Undercut,
                              B=Undercut_Offset,
                              operation='A-B',
                              layer=res_params.elbow_undercut_layer)
        
        Elbow_Left_Undercut << Undercut

        return Elbow_Left_Undercut





