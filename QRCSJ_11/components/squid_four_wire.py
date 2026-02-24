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

from components.squid import Squid, SquidParams
from components.junction_two_wire import JJ2Wire
from components.junction_four_wire import JJ4Wire, PadParams
from components.resistor import Resistor, ResParams

class Squid4Wire:

    def __init__(self) -> None:
        self.squid_params: Optional[SquidParams] = None
        self.pad_params: Optional[PadParams] = None
        self.res_params: Optional[ResParams] = None

        self.device: Optional[Device] = None

        self.squid: Optional[Squid] = None
        self.squid_ref: Optional[DeviceReference] = None

        self.top_left_resistor: Optional[DeviceReference] = None
        self.top_right_resistor: Optional[DeviceReference] = None
        self.bot_left_resistor: Optional[DeviceReference] = None
        self.bot_right_resistor: Optional[DeviceReference] = None
        
        self.top_left_pad: Optional[DeviceReference] = None
        self.top_right_pad: Optional[DeviceReference] = None
        self.bot_left_pad: Optional[DeviceReference] = None
        self.bot_right_pad: Optional[DeviceReference] = None


    def create_resistors(self, res_params: ResParams) -> None:

        TopLeftResistor, TopRightResistor = JJ4Wire.create_top_resistors(res_params)
        BotLeftResistor, BotRightResistor = JJ4Wire.create_bot_resistors(res_params)

        self.top_left_resistor, self.top_right_resistor, self.bot_left_resistor, self.bot_right_resistor = self.device << [TopLeftResistor, TopRightResistor, BotLeftResistor, BotRightResistor]

    def position_resistors(self) -> None:
        top_resistors: Group = self.top_left_resistor + self.top_right_resistor
        top_resistors.move(origin=self.top_left_resistor.ports['bot'], destination=self.squid_ref.ports['top'])

        bot_resistors: Group = self.bot_left_resistor + self.bot_right_resistor
        bot_resistors.move(origin=self.bot_left_resistor.ports['bot'], destination=self.squid_ref.ports['bot'])

    def create_pads(self, pad_params:PadParams) -> None:
        Pad = JJ2Wire.create_pad(pad_params)

        self.top_left_pad, self.top_right_pad, self.bot_left_pad, self.bot_right_pad = self.device << [Pad, Pad, Pad, Pad]

    def position_pads(self, pad_params: PadParams) -> None:

        self.top_left_pad.connect('E', self.top_left_resistor.ports['top'])
        self.top_left_pad.move((pad_params.overlap_x, pad_params.pad_size/2 - pad_params.overlap_y))
        self.top_right_pad.connect('W', self.top_right_resistor.ports['bot'])
        self.top_right_pad.move((-pad_params.overlap_x, pad_params.pad_size/2 - pad_params.overlap_y))

        self.bot_left_pad.connect('E', self.bot_left_resistor.ports['top'])
        self.bot_left_pad.move((pad_params.overlap_x, -pad_params.pad_size/2 + pad_params.overlap_y))
        self.bot_right_pad.connect('W', self.bot_right_resistor.ports['bot'])
        self.bot_right_pad.move((-pad_params.overlap_x, -pad_params.pad_size/2 + pad_params.overlap_y))

    def generate_squid_four_wire(self, squid_params: SquidParams, res_params: ResParams, pad_params: PadParams, writefield_params: WritefieldParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Squid for four wire measurement already exists.')
        else:
            self.device= Device('Junction four wire')

            self.squid = Squid()
            self.squid.generate_squid(squid_params)
            self.squid_ref = self.device << self.squid.device
            

            self.create_resistors(res_params)
            self.position_resistors()

            utils.unify_layer(self.device, res_params.undercut_layer)
            utils.unify_layer(self.device, res_params.ebeam_layer)
            utils.subtract_layers(self.device, layer_a=res_params.undercut_layer, layer_b=res_params.ebeam_layer, offset=res_params.undercut_spacing)

            utils.add_writefield(writefield_params, self.device)

            self.create_pads(pad_params)
            self.position_pads(pad_params)

