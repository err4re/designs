### phidl imports
from phidl import Device, Layer, LayerSet, Group
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

from components.junction_two_wire import JJ2Wire, PadParams
from components.resistor import Resistor, ResParams

class Resistor2Wire:

    def __init__(self) -> None:

        self.res_params: Optional[ResParams] = None
        self.pad_params: Optional[PadParams] = None
        self.writefield_params: Optional[WritefieldParams]

        self.device: Optional[Device] = None

        self.resistor: Optional[Resistor] = None
        self.resistor_ref: Optional[DeviceReference] = None

        self.top_pad: Optional[DeviceReference] = None
        self.bot_pad: Optional[DeviceReference] = None


    def generate_resistor_two_wire(self, res_params: ResParams, pad_params: PadParams, writefield_params:WritefieldParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Resistor for two wire measurement already exists.')
        else:
            self.device= Device('Resistor two wire')

            self.resistor = Resistor()
            self.resistor.generate_resistor(res_params)
            self.resistor_ref = self.device << self.resistor.device

            utils.add_writefield(writefield_params, self.device)

            self.top_pad = self.device << JJ2Wire.create_pad(pad_params)
            self.bot_pad = self.device << JJ2Wire.create_pad(pad_params)

            self.top_pad.connect('E', self.resistor_ref.ports['top'])
            self.bot_pad.connect('E', self.resistor_ref.ports['bot'])

            self.top_pad.movey(-pad_params.overlap_y+pad_params.pad_size/2)
            self.top_pad.movex(pad_params.overlap_x)
            self.bot_pad.movey(pad_params.overlap_y-pad_params.pad_size/2)
            self.bot_pad.movex(pad_params.overlap_x)

            self.res_params = res_params
            self.pad_params = pad_params

        