### phidl imports
from phidl import Device, Layer, LayerSet
from phidl.device_layout import DeviceReference
from phidl import quickplot as qp

import phidl.geometry as pg
import phidl.utilities as pu
import phidl.routing as pr
import phidl.path as pp

### general python imports
import numpy as np

from dataclasses import dataclass, field, replace
from typing import Tuple, Optional

import itertools


from components.default_layerset import default_ls
from components import utils
from components.utils import WritefieldParams

from components.junction import JJ, JJParams
from components.squid import Squid, SquidParams
from components.junction_two_wire import JJ2Wire, PadParams



class Squid2Wire:

    def __init__(self) -> None:
        self.squid_params: Optional[JJParams] = None
        self.pad_params: Optional[PadParams] = None

        self.device: Optional[Device] = None

        self.squid: Optional[Squid] = None
        self.squid_ref: Optional[DeviceReference] = None
        
        self.top_pad: Optional[DeviceReference] = None
        self.bot_pad: Optional[DeviceReference] = None

    
    def generate_squid_two_wire(self, squid_params: SquidParams, pad_params: PadParams, writefield_params: WritefieldParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Junction for two wire measurement already exists.')
        else:
            self.device= Device('Junction two wire')

            self.squid = Squid()
            self.squid.generate_squid(squid_params)
            self.squid_ref = self.device << self.squid.device

            utils.add_writefield(writefield_params, self.device)

            self.top_pad = self.device << JJ2Wire.create_pad(pad_params)
            self.bot_pad = self.device << JJ2Wire.create_pad(pad_params)

            self.top_pad.connect('S', self.squid_ref.ports['top'])
            self.bot_pad.connect('N', self.squid_ref.ports['bot'])

            self.top_pad.movey(-pad_params.overlap_y)
            self.bot_pad.movey(pad_params.overlap_y)

            self.jj_params = squid_params
            self.pad_params = pad_params

