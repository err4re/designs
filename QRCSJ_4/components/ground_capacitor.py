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


### define parameters
@dataclass
class GroundCapaParams:
    distance_to_ground: float = 100

    arm_width: float = 3

    pad_size: float = 3

    capa_width: float = 10
    capa_height: float = 3

    orientation: float = 90

    layer: Layer = default_ls['routing']


class GroundCapa:

    def __init__(self) -> None:
        self.capa_params: Optional[GroundCapaParams] = None

        self.device: Optional[Device] = None

        self.pad: Optional[DeviceReference] = None
        self.arm: Optional[DeviceReference] = None
        self.plate: Optional[DeviceReference] = None

    @staticmethod
    def create_pad(capa_params: GroundCapaParams) -> Device:
        Pad = Device('Pad')
        Pad << pg.rectangle(size=(capa_params.pad_size, capa_params.pad_size), layer=capa_params.layer)

        Pad.add_port(name='ground', midpoint=Pad.center, width=capa_params.pad_size, orientation=capa_params.orientation)

        return Pad
    
    @staticmethod
    def create_arm(capa_params: GroundCapaParams) -> Device:
        Arm = Device('Arm')
        Arm << pg.rectangle(size=(capa_params.arm_width, -(capa_params.distance_to_ground - capa_params.pad_size/2 - capa_params.capa_height)), layer=capa_params.layer)

        return Arm
    
    @staticmethod
    def create_capa_plate(capa_params: GroundCapaParams) -> Device:
        Plate = Device('Capacitor plate')
        Plate << pg.rectangle(size=(capa_params.capa_width, capa_params.capa_height), layer=capa_params.layer)

        Plate.add_port(name='capa', midpoint=((Plate.xmax + Plate.xmin)/2, Plate.ymin), width=capa_params.capa_width, orientation=capa_params.orientation)

        return Plate
    
    def generate_ground_capa(self, capa_params: GroundCapaParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Capacitor to ground already exists.')
        else:
            self.device = Device('Capacitor to ground')

            self.pad = self.device << self.create_pad(capa_params)
            self.pad.move((-capa_params.pad_size/2, -capa_params.pad_size/2))

            self.arm = self.device << self.create_arm(capa_params)
            self.arm.move((-capa_params.arm_width/2, -capa_params.pad_size/2))

            self.plate = self.device << self.create_capa_plate(capa_params)
            self.plate.move((-capa_params.capa_width/2, self.arm.ymin - capa_params.capa_height))

            self.capa_params = capa_params




