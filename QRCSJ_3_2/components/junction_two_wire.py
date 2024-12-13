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


@dataclass
class PadParams:
    pad_size: float = 120
    spacing: float = 100  
    layer: Layer = default_ls['ebeam_strong']
    overlap_x: float = 4
    overlap_y: float = 4  

    connector_width: float = 2

    name: str = '00'
    text_size: float = 25

    writefield_params: WritefieldParams = WritefieldParams(writefield_height=200, 
                                                           writefield_width=200, 
                                                           writefield_layer=default_ls['writefield_ebeam_strong'],
                                                           working_area_layer=default_ls['working_area_ebeam_strong'],
                                                           adapt_working_area_size=True)

    



class JJ2Wire:

    def __init__(self) -> None:
        self.jj_params: Optional[JJParams] = None
        self.pad_params: Optional[PadParams] = None
        self.writefield_params: Optional[WritefieldParams] = None

        self.device: Optional[Device] = None

        self.junction: Optional[JJ] = None
        self.junction_ref: Optional[DeviceReference] = None
        
        self.top_pad: Optional[DeviceReference] = None
        self.bot_pad: Optional[DeviceReference] = None

    @staticmethod
    def create_pad(pad_params: PadParams) -> Device:
        Pad = Device('Pad')

        Square = pg.compass(size=(pad_params.pad_size, pad_params.pad_size), layer=pad_params.layer)
        #Pad = pg.compass(size=(pad_params.pad_size, pad_params.pad_size), layer=pad_params.layer)
        Pad << pg.boolean(Square, pg.text(text=pad_params.name, size=pad_params.text_size, justify='center', layer=pad_params.layer).move(Square.center), operation='A-B', layer=pad_params.layer)

        [Pad.add_port(port) for port in Square.get_ports()]

        utils.add_writefield(pad_params.writefield_params, Pad)
        
        return Pad
    
    def generate_junction_two_wire(self, jj_params: JJParams, pad_params: PadParams, writefield_params: WritefieldParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Junction for two wire measurement already exists.')
        else:
            self.device= Device('Junction two wire')

            self.junction = JJ()
            self.junction.generate_jj(jj_params)
            self.junction_ref = self.device << self.junction.device

            utils.add_writefield(writefield_params, self.device)

            self.top_pad = self.device << JJ2Wire.create_pad(pad_params)
            self.bot_pad = self.device << JJ2Wire.create_pad(pad_params)

            self.top_pad.connect('S', self.junction_ref.ports['top'])
            self.bot_pad.connect('N', self.junction_ref.ports['bot'])

            self.top_pad.movey(-pad_params.overlap_y)
            self.bot_pad.movey(pad_params.overlap_y)

            self.jj_params = jj_params
            self.pad_params = pad_params
            self.writefield_params = writefield_params

