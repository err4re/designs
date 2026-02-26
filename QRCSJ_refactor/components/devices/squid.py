### phidl imports
from phidl import Device, Layer, LayerSet
from phidl.device_layout import DeviceReference, Polygon
from phidl import quickplot as qp

import phidl.geometry as pg
import phidl.utilities as pu
import phidl.routing as pr
import phidl.path as pp

import gdspy

### Dolan junction
from .josephson_junction import DolanJunction, DolanJunctionParams

### general python imports
import numpy as np

from typing import Optional
from dataclasses import dataclass

### default layerset
from ..layerset.defaults import DEFAULT_DESIGN_LAYER_SET

default_design_ls = DEFAULT_DESIGN_LAYER_SET
default_ls = default_design_ls.layer_set


### define SQUID parameters
@dataclass(frozen=True)
class SquidParams:

    junction_left: Optional[DolanJunction] = None
    junction_right: Optional[DolanJunction] = None

    island_width: float = 20.22
    island_height: float = 1.2

    overlap: float = 0.1

    shorted: bool = False
    short_width: float = 0.22
    short_spacing: float = 13
    short_side: str = 'left'

    undercut_offset = 0.08
    undercut_width = 0.8

    ### Layers ###
    ##############
    island_layer: Layer = default_ls['ebeam_low']
    short_layer: Layer = default_ls['ebeam']

    undercut_layer: Layer = default_ls['undercut']

    def __post_init__(self):


        # validate junctions
        if (self.junction_left.device is None) or (self.junction_right.device is None):
            raise ValueError(f'Junctions not built yet, junction_left is {self.junction_left.device} and junction_right is {self.junction_right.device}!')


        if self.junction_left.params.junction_total_length != self.junction_right.params.junction_total_length:
            raise ValueError(f'Junctions should be equal total length, but junction_left is {self.junction_left.params.junction_total_length} and junction_right is {self.junction_right.params.junction_total_length} long!')

        # validate short side
        valid_short_side = ['left', 'right']
        if self.short_side not in valid_short_side:
            raise ValueError(f'Unsupported short side: {self.short_side}')


### define SQUID class
class Squid:

    def __init__(self) -> None:
        
        ### phidl device containing the full squid
        self.device: Optional[Device] = None

        # parameters used to build squid
        self.params: Optional[SquidParams] = None


        ### all elements making up full squid

        # both junctions making up the squid
        self.Junction_Left: Optional[Device] = None
        self.Junction_Right: Optional[Device] = None

        self.junction_left_ref: Optional[DeviceReference] = None
        self.junction_right_ref: Optional[DeviceReference] = None

        # both islands
        self.Island_Top: Optional[Device] = None
        self.Island_Top_Undercut: Optional[Device] = None
        self.Island_Bot: Optional[Device] = None
        self.Island_Bot_Undercut: Optional[Device] = None

        self.island_top_ref: Optional[DeviceReference] = None
        self.island_top_undercut_ref: Optional[DeviceReference] = None
        self.island_bot_ref: Optional[DeviceReference] = None
        self.island_bot_undercut_ref: Optional[DeviceReference] = None

        # short
        self.Short: Optional[Device] = None

        self.short_ref: Optional[DeviceReference] = None


    def build_squid(self, squid_params: SquidParams) -> Device:

        if self.device is not None:
            raise RuntimeWarning('This Squid has already been built and can by accessed through self.device!')
        
        Squid_Device = Device('squid')

        # build top island
        self.Island_Top = Squid._build_top_island(squid_params)
        self.island_top_ref = Squid_Device << self.Island_Top

        # get junctions from squid params
        self.Junction_Left = squid_params.junction_left.device        
        self.junction_left_ref  = Squid_Device << self.Junction_Left

        self.Junction_Right = squid_params.junction_right.device
        self.junction_right_ref = Squid_Device << self.Junction_Right

        # connect junctions to top island
        self.junction_left_ref.connect(port=self.junction_left_ref.ports['top'], destination=self.Island_Top.ports['junction_left'])
        self.junction_right_ref.connect(port=self.junction_right_ref.ports['top'], destination=self.Island_Top.ports['junction_right'])

        # build bot island
        self.Island_Bot = Squid._build_bot_island(squid_params)
        self.island_bot_ref = Squid_Device << self.Island_Bot

        # connect bot island to junctions
        self.island_bot_ref.connect(port=self.island_bot_ref.ports['junction_left'], destination=self.junction_left_ref.ports['bot'])

        if squid_params.shorted:
            # build short
            self.Short  = Squid._build_short(squid_params)
            self.short_ref = Squid_Device << self.Short

            # connect short to top island
            self.short_ref.connect(port=self.short_ref.ports['top'], destination=self.island_top_ref.ports['short'])

        # build undercuts for islands
        self.Island_Top_Undercut = self._build_top_island_undercut(Squid_Device, squid_params)
        self.Island_Bot_Undercut = self._build_bot_island_undercut(Squid_Device, squid_params)

        self.island_top_undercut_ref = Squid_Device << self.Island_Top_Undercut
        self.island_bot_undercut_ref = Squid_Device << self.Island_Bot_Undercut
        

        # save device and params that were used to generate it
        self.device = Squid_Device
        self.params = squid_params

        return Squid_Device



    @staticmethod
    def _build_short(squid_params: SquidParams) -> Device:

        Short = Device('squid_short')

        compass_ref = Short << pg.compass(size=(squid_params.short_width, squid_params.junction_left.params.junction_total_length + 2*squid_params.overlap), layer=squid_params.short_layer)

        Short.add_port(name='top', midpoint=compass_ref.ports['N'].midpoint - [0, squid_params.overlap], orientation=compass_ref.ports['N'].orientation)
        Short.add_port(name='bot', midpoint=compass_ref.ports['S'].midpoint + [0, squid_params.overlap], orientation=compass_ref.ports['S'].orientation)

        return Short
    
    @staticmethod
    def _build_top_island(squid_params: SquidParams) -> Device:

        Island = Device('squid_top_island')

        # width of island depends on short, with short it is extended by short_spacing 
        if squid_params.shorted:
            Island << pg.rectangle(size=(squid_params.island_width+squid_params.short_spacing, squid_params.island_height), layer=squid_params.island_layer)

            # position of ports depends on the side chosen for the short
            if squid_params.short_side == 'left':
                Island.add_port(name='short',
                                midpoint=(squid_params.short_width/2, 0),
                                width=squid_params.short_width,
                                orientation=-90
                                )
                Island.add_port(name='junction_left',
                                midpoint=(squid_params.short_spacing + squid_params.junction_left.params.arm_width/2, 0),
                                width=squid_params.junction_left.params.arm_width,
                                orientation=-90
                                )
                Island.add_port(name='junction_right',
                                midpoint=(squid_params.short_spacing + squid_params.island_width - squid_params.junction_right.params.arm_width/2, 0),
                                width=squid_params.junction_right.params.arm_width,
                                orientation=-90
                                )
                
            elif squid_params.short_side == 'right':
                Island.add_port(name='junction_left',
                                midpoint=(squid_params.junction_left.params.arm_width/2, 0),
                                width=squid_params.junction_left.params.arm_width,
                                orientation=-90
                                )
                Island.add_port(name='junction_right',
                                midpoint=(squid_params.island_width - squid_params.junction_right.params.arm_width/2, 0),
                                width=squid_params.junction_right.params.arm_width,
                                orientation=-90
                                )
                Island.add_port(name='short',
                                midpoint=(squid_params.island_width + squid_params.short_spacing - squid_params.short_width/2, 0),
                                width=squid_params.short_width,
                                orientation=-90
                                )
            else:
                raise NotImplementedError(f'Short side {squid_params.short_side} is valid but not implemented yet!')

        # for squid without short island width is set by island width and position of ports is fixed
        else:
            Island << pg.rectangle(size=(squid_params.island_width, squid_params.island_height), layer=squid_params.island_layer)

            Island.add_port(name='junction_left',
                            midpoint=(squid_params.junction_left.params.arm_width/2, 0),
                            width=squid_params.junction_left.params.arm_width,
                            orientation=-90
                            )
            Island.add_port(name='junction_right',
                            midpoint=(squid_params.island_width - squid_params.junction_right.params.arm_width/2, 0),
                            width=squid_params.junction_right.params.arm_width,
                            orientation=-90
                            )

        Island.add_port(name='top',
                        midpoint=(Island.ports['junction_left'].midpoint + [squid_params.island_width/2 - Island.ports['junction_left'].width/2, squid_params.island_height]),
                        orientation=90)

        return Island
    

    def _build_top_island_undercut(self, Squid_Device:  Device, squid_params: SquidParams) -> Device:
        """
        Build undercut for top island. Needs Island_Top and will respect undercut offset with respect to all elements already built into the squid at self.device.
        """

        if self.Island_Top is None:
            raise RuntimeError(f'Top island has not been built yet, self.Top_Island is {self.Island_Top}!')

        Island_Top_Undercut = Device('island_top_undercut')

        Undercut = pg.offset(elements=[self.island_bot_ref], distance=squid_params.undercut_width)
        Undercut_Offset = pg.offset(elements=[Squid_Device], distance=squid_params.undercut_offset)

        Undercut = pg.boolean(A=Undercut,
                              B=Undercut_Offset,
                              operation='A-B',
                              layer=squid_params.undercut_layer)
        
        Island_Top_Undercut << Undercut

        return Island_Top_Undercut


    def _build_bot_island_undercut(self, Squid_Device:  Device, squid_params: SquidParams) -> Device:
        """
        Build undercut for bot island. Needs Island_Bot and will respect undercut offset with respect to all elements already built into the squid at self.device.
        """

        if self.Island_Bot is None:
            raise RuntimeError(f'Bot island has not been built yet, self.Bot_Island is {self.Island_Bot}!')

        Island_Bot_Undercut = Device('island_bot_undercut')

        Undercut = pg.offset(elements=[self.island_top_ref], distance=squid_params.undercut_width)
        Undercut_Offset = pg.offset(elements=[Squid_Device], distance=squid_params.undercut_offset)

        Undercut = pg.boolean(A=Undercut,
                              B=Undercut_Offset,
                              operation='A-B',
                              layer=squid_params.undercut_layer)
        
        Island_Bot_Undercut << Undercut

        return Island_Bot_Undercut


    
    @staticmethod
    def _build_bot_island(squid_params: SquidParams) -> Device:

        Island = Device('squid_bot_island')

        if squid_params.shorted:
            Island << pg.rectangle(size=(squid_params.island_width+squid_params.short_spacing, squid_params.island_height), layer=squid_params.island_layer)

            if squid_params.short_side == 'left':
                Island.add_port(name='short',
                                midpoint=(squid_params.short_width/2, squid_params.island_height),
                                width=squid_params.short_width,
                                orientation=90
                                )
                Island.add_port(name='junction_left',
                                midpoint=(squid_params.short_spacing + squid_params.junction_left.params.arm_width/2, squid_params.island_height),
                                width=squid_params.junction_left.params.arm_width,
                                orientation=90
                                )
                Island.add_port(name='junction_right',
                                midpoint=(squid_params.short_spacing + squid_params.island_width - squid_params.junction_right.params.arm_width/2, squid_params.island_height),
                                width=squid_params.junction_right.params.arm_width,
                                orientation=90
                                )
                
            elif squid_params.short_side == 'right':
                Island.add_port(name='junction_left',
                                midpoint=(squid_params.junction_left.params.arm_width/2, squid_params.island_height),
                                width=squid_params.junction_left.params.arm_width,
                                orientation=90
                                )
                Island.add_port(name='junction_right',
                                midpoint=(squid_params.island_width - squid_params.junction_right.params.arm_width/2, squid_params.island_height),
                                width=squid_params.junction_right.params.arm_width,
                                orientation=90
                                )
                Island.add_port(name='short',
                                midpoint=(squid_params.island_width + squid_params.short_spacing - squid_params.short_width/2, squid_params.island_height),
                                width=squid_params.short_width,
                                orientation=90
                                )
            else:
                raise NotImplementedError(f'Short side {squid_params.short_side} is valid but not implemented yet!')

        else:
            Island.add_port(name='junction_left',
                            midpoint=(squid_params.junction_left.params.arm_width/2, squid_params.island_height),
                            width=squid_params.junction_left.params.arm_width,
                            orientation=90
                            )
            Island.add_port(name='junction_right',
                            midpoint=(squid_params.island_width - squid_params.junction_right.params.arm_width/2, squid_params.island_height),
                            width=squid_params.junction_right.params.arm_width,
                            orientation=90
                            )

        Island.add_port(name='bot',
                        midpoint=(Island.ports['junction_left'].midpoint + [squid_params.island_width/2 - Island.ports['junction_left'].width/2, - squid_params.island_height]),
                        orientation=-90)

        return Island






    


