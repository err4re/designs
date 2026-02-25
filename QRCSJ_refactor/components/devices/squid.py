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
from josephson_junction import DolanJunction, DolanJunctionParams

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

    shorted: bool = False
    short
    short_spacing: float = 13
    short_side: str = 'left'

    ### Layers ###
    ##############
    island_layer: Layer = default_ls['ebeam_low']
    arm_layer: Layer = default_ls['ebeam']

    undercut_layer: Layer = default_ls['undercut']

    def __post_init__(self):

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


    def build_squid(self, squid_params: SquidParams) -> Device:



    def _build_short(squid_params) -> 