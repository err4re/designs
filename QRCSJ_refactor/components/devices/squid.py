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

    junction_1: Optional[DolanJunction] = None
    junction_2: Optional[DolanJunction] = None

    island_width: float = 20.22
    island_height: float = 1.2

    overlap: float = 0.1

    shorted: bool = False
    short_width: float = 0.22
    short_spacing: float = 13
    short_side: str = 'left'

    ### Layers ###
    ##############
    island_layer: Layer = default_ls['ebeam_low']
    short_layer: Layer = default_ls['ebeam']

    undercut_layer: Layer = default_ls['undercut']

    def __post_init__(self):

        # validate junctions
        if self.junction_1.params.junction_total_length != self.junction_2.params.junction_total_length:
            raise ValueError(f'Junctions should be equal total length, but junction_1 is {self.junction_1.params.junction_total_length} and junction_2 is {self.junction_2.params.junction_total_length} long!')

        # validate short side
        valid_short_side = ['left', 'right']
        if self.short_side not in valid_short_side:
            raise ValueError(f'Unsupported short side: {self.short_side}')


### define SQUID class
class Squid:

    def __init__(self) -> None:
        
        ### phidl device containing the full squid
        self.device: Optional[Device] = None


        ### all elements making up full squid

        # parameters used to build squid
        self.params: Optional[SquidParams] = None

        # both junctions making up the squid
        self.Junction_1: Optional[Device] = None
        self.Junction_2: Optional[Device] = None

        self.junction_1_ref: Optional[DeviceReference] = None
        self.junction_2_ref: Optional[DeviceReference] = None

        # both islands
        self.Island_1: Optional[Device] = None
        self.Island_1_Undercut: Optional[Device] = None
        self.Island_2: Optional[Device] = None
        self.Island_2_Undercut: Optional[Device] = None

        self.island_1_ref: Optional[DeviceReference] = None
        self.island_1_undercut_ref: Optional[DeviceReference] = None
        self.island_2_ref: Optional[DeviceReference] = None
        self.island_2_undercut_ref: Optional[DeviceReference] = None

        # short
        self.Short: Optional[Device] = None

        self.short_ref: Optional[DeviceReference] = None


    # def build_squid(self, squid_params: SquidParams) -> Device:



    def _build_short(squid_params: SquidParams) -> Device:

        Short = Device('squid_short')

        Short << pg.compass(size=(squid_params.short_width, squid_params.junction_1.params.junction_total_length + 2*squid_params.overlap), layer=squid_params.short_layer)

        Short.add_port(name='in', midpoint=Short.ports['N'].midpoint - [0, squid_params.overlap], orientation=Short.ports['N'].orientation)
        Short.add_port(name='out', midpoint=Short.ports['S'].midpoint + [0, squid_params.overlap], orientation=Short.ports['S'].orientation)

        return Short
    
    def _build_top_island(squid_params: SquidParams) -> Device:

        Island = Device('squid_island')

        if squid_params.shorted:
            Island << pg.rectangle(size=(squid_params.island_width+squid_params.short_spacing, squid_params.island_height), layer=squid_params.island_layer)

            if squid_params.short_side == 'left':
                Island.add_port(name='short',
                                midpoint=(squid_params.short_width/2, 0),
                                width=squid_params.short_width,
                                orientation=-90
                                )
                Island.add_port(name='junction_1',
                                midpoint=(squid_params.short_spacing + squid_params.junction_1.params.arm_width/2, 0),
                                width=squid_params.junction_1.params.arm_width,
                                orientation=-90
                                )
                Island.add_port(name='junction_2',
                                midpoint=(squid_params.short_spacing + squid_params.island_width - squid_params.junction_2.params.arm_width/2, 0),
                                width=squid_params.junction_2.params.arm_width,
                                orientation=-90
                                )
                
            elif squid_params.short_side == 'right':
                Island.add_port(name='junction_1',
                                midpoint=(squid_params.junction_1.params.arm_width/2, 0),
                                width=squid_params.junction_1.params.arm_width,
                                orientation=-90
                                )
                Island.add_port(name='junction_2',
                                midpoint=(squid_params.island_width - squid_params.junction_2.params.arm_width/2, 0),
                                width=squid_params.junction_2.params.arm_width,
                                orientation=-90
                                )
                Island.add_port(name='short',
                                midpoint=(squid_params.island_width + squid_params.short_spacing - squid_params.short_width/2, 0),
                                width=squid_params.short_width,
                                orientation=-90
                                )
            else:
                raise NotImplementedError(f'Short side {squid_params.short_side} is valid but not implemented yet!')

        else:
            Island.add_port(name='junction_1',
                            midpoint=(squid_params.junction_1.params.arm_width/2, 0),
                            width=squid_params.junction_1.params.arm_width,
                            orientation=-90
                            )
            Island.add_port(name='junction_2',
                            midpoint=(squid_params.island_width - squid_params.junction_2.params.arm_width/2, 0),
                            width=squid_params.junction_2.params.arm_width,
                            orientation=-90
                            )

        Island.add_port(name='in',
                        midpoint=(Island.ports['junction_1'].midpoint + [squid_params.island_width/2 - Island.ports['junction_1'].width/2, squid_params.island_height]),
                        orientation=90)

        return Island






    


