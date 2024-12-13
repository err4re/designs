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

from components.junction_two_wire import JJ2Wire, PadParams
from components.squid import Squid, SquidParams


class ContactResistance:

    def __init__(self) -> None:
        self.squid_params: Optional[SquidParams] = None
        self.pad_params: Optional[PadParams] = None
        self.writefield_params: Optional[WritefieldParams] = None

        self.device: Optional[Device] = None

        self.left_lead_ref: Optional[DeviceReference] = None
        self.right_lead_ref: Optional[DeviceReference] = None

        self.contact_island_ref: Optional[DeviceReference] = None
        
        self.left_pad: Optional[DeviceReference] = None
        self.right_pad: Optional[DeviceReference] = None

    @staticmethod
    def create_lead(length: float, squid_params: SquidParams) -> Device:

        

        Lead = pg.compass(size=(squid_params.arm_width, length), layer=squid_params.layer)
        # Undercut = pg.rectangle(size=(2*squid_params.arm_width, squid_params.undercut_width), layer=squid_params.undercut_layer)

        # Undercut.move((-squid_params.arm_width, length/2))

        # Lead << Undercut

        # utils.subtract_layers(Lead, squid_params.undercut_layer, squid_params.layer, offset=squid_params.undercut_spacing)

        return Lead
    
    @staticmethod
    def create_contact_island(length: float, squid_params: SquidParams) -> Device:

        Island = pg.compass(size=(length, squid_params.island_height), layer=squid_params.layer)

        Undercut = pg.rectangle(size=(length+squid_params.undercut_width+2*squid_params.arm_width, squid_params.island_height+2*squid_params.undercut_width))
        Undercut.move((-length/2-squid_params.undercut_width-squid_params.arm_width, -squid_params.island_height/2 - squid_params.undercut_width))

        Undercut = pg.boolean(A=Undercut, 
                              B=pg.offset(Island, distance=squid_params.undercut_spacing),
                              operation='A-B',
                              layer=squid_params.undercut_layer)

        Island << Undercut

        return Island



    def generate_contact_resistance(self, squid_params: SquidParams, pad_params: PadParams, writefield_params: WritefieldParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Structure to measure contact resistance already exists.')
        else:
            self.device= Device('Contact resistance')

            self.left_lead_ref = self.device << ContactResistance.create_lead(length=squid_params.total_length, squid_params=squid_params)
            self.right_lead_ref = self.device << ContactResistance.create_lead(length=squid_params.total_length, squid_params=squid_params).movex(squid_params.island_width+squid_params.arm_width)

            utils.add_writefield(writefield_params, self.device)

            self.left_pad = self.device << JJ2Wire.create_pad(pad_params)
            self.right_pad = self.device << JJ2Wire.create_pad(pad_params)

            self.left_pad.connect('N', self.left_lead_ref.ports['S'])
            self.right_pad.connect('N', self.right_lead_ref.ports['S'])

            self.left_pad.movey(pad_params.overlap_y)
            self.right_pad.movey(pad_params.overlap_y)

            self.left_pad.movex(-pad_params.pad_size/2 + pad_params.overlap_x)
            self.right_pad.movex(pad_params.pad_size/2 - pad_params.overlap_x)

            self.contact_island_ref = self.device << ContactResistance.create_contact_island(length=squid_params.island_width, squid_params=squid_params)

            self.contact_island_ref.connect('W', self.left_lead_ref.ports['E'])

            self.contact_island_ref.movey(squid_params.total_length/2 - squid_params.island_height/2)

            utils.subtract_layers(self.device, squid_params.undercut_layer, squid_params.layer, offset=squid_params.undercut_spacing)


            self.squid_params = squid_params
            self.pad_params = pad_params
            self.writefield_params = writefield_params


    



