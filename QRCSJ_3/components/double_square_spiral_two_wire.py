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

from components.junction_two_wire import JJ2Wire, PadParams,WritefieldParams
from components.double_square_spiral import DoubleSquareSpiral, SquareSpiral, SquareSpiralParams

class DoubleSquareSpiral2Wire:

    def __init__(self) -> None:

        self.spiral_params: Optional[SquareSpiralParams] = None
        self.pad_params: Optional[PadParams] = None

        self.device: Optional[Device] = None

        self.double_square_spiral: Optional[DoubleSquareSpiral] = None
        self.double_square_spiral_ref: Optional[DeviceReference] = None

        self.top_pad: Optional[DeviceReference] = None
        self.bot_pad: Optional[DeviceReference] = None


    def generate_double_square_spiral_two_wire(self, spiral_params: SquareSpiralParams, pad_params: PadParams, writefield_params: WritefieldParams, overwrite: bool = False) -> None:

        if (self.device is not None) and not overwrite:
            print('Double square spiral for two wire measurement already exists.')
        else:
            self.device = Device('Doubly wound square spiral two wire')

            self.double_square_spiral = DoubleSquareSpiral()
            self.double_square_spiral.generate_double_square_spiral(spiral_params)
            self.double_square_spiral_ref = self.device << self.double_square_spiral.device

            utils.add_writefield(writefield_params, self.device)

            self.top_pad = self.device << JJ2Wire.create_pad(pad_params)
            self.bot_pad = self.device << JJ2Wire.create_pad(pad_params)

            self.top_pad.connect('S', self.double_square_spiral_ref.ports['top'])
            self.bot_pad.connect('N', self.double_square_spiral_ref.ports['bot'])

            self.top_pad.movey(-pad_params.overlap_y)
            self.bot_pad.movey(pad_params.overlap_y)

            self.spiral_params = spiral_params
            self.pad_params = pad_params





