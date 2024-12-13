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

from components.squid import Squid, SquidParams
from components.resistor import Resistor, ResParams
from components.junction_resistor import JJResistor, CapaParams


class SquidResistor():

    def __init__(self) -> None:
        self.res_params: Optional[ResParams] = None
        self.squid_params: Optional[SquidParams] = None
        self.writefield_params: Optional[WritefieldParams] = None

        self.device: Optional[Device] = None

        self.squid: Optional[Squid] = None
        self.resistor: Optional[Resistor] = None

        self.squid_ref: Optional[DeviceReference] = None
        self.shunt_resistor: Optional[DeviceReference] = None

        self.top_connector: Optional[Device] = None
        self.bot_connector: Optional[Device] = None



    @staticmethod
    def sync_parameters(res_params: ResParams, squid_params: SquidParams, capa_params: CapaParams) -> Tuple[ResParams, SquidParams, CapaParams]:
        squid_params = replace(squid_params)

        if res_params.small_resistor:
            squid_params.total_length = res_params.total_spacing+(res_params.num_segments)*res_params.resistor_width+2*capa_params.length_y + res_params.spacing + res_params.connector_height
        elif res_params.connectors:
            squid_params.total_length = res_params.total_spacing+(res_params.num_segments-2)*res_params.resistor_width+2*res_params.connector_height+2*capa_params.length_y
        else:
            squid_params.total_length = res_params.total_spacing+(res_params.num_segments)*res_params.resistor_width+2*capa_params.length_y

        # make sure the squid loop is smaller than the total length
        if squid_params.total_length < squid_params.loop_height:
            squid_params.total_length = squid_params.loop_height + 2*squid_params.island_height + 2*capa_params.length_y


        capa_params = replace(capa_params)
        utils.sync_attributes(res_params, capa_params)
        capa_params.arm_width = squid_params.arm_width

        return res_params, squid_params, capa_params
    
    def add_ports(self) -> None:
        self.device.add_port(name= 'capa top', port=self.top_connector.ports['capa out'])
        self.device.add_port(name= 'capa bot', port=self.bot_connector.ports['capa out'])



    def generate_squid_resistor(self, res_params: ResParams, squid_params: SquidParams, capa_params: CapaParams, writefield_params:WritefieldParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Squid with shunt resistor already exists.')
        else:
            self.device = Device('Squid with resistive shunt')

            # synch parameters, derive parameters from resistor parameters
            res_params, squid_params, capa_params = SquidResistor.sync_parameters(res_params, squid_params, capa_params)

            self.resistor = Resistor()
            self.resistor.generate_resistor(res_params)

            self.squid = Squid()
            self.squid.generate_squid(squid_params)
            self.squid_ref = self.device << self.squid.device

            Connector = JJResistor.create_connector(capa_params)

            # fix overlapping undercut
            Connector.move(origin=Connector.ports['in'], destination=self.squid_ref.ports['top'])
            utils.subtract_overlap_from_layer(Connector, self.squid_ref, res_params.undercut_layer, res_params.undercut_spacing)

            self.top_connector = self.device << Connector        
            self.bot_connector = self.device << Connector

            # put bot connector in the right place
            self.bot_connector.mirror(p1=(-1,0), p2=(1,0))

            # move resistor to final position
            if res_params.small_resistor and not res_params.connectors:
                self.resistor.device.move(origin=self.resistor.device.ports['top left small'], destination=self.top_connector.ports['out'])
                self.resistor.device.movex(res_params.arm_width/2 - capa_params.arm_width/2)

            else:
                self.resistor.device.move(origin=self.resistor.device.ports['top'], destination=self.top_connector.ports['out'])
                self.resistor.device.move((-capa_params.arm_width/2,-res_params.connector_height/2))

            self.add_ports()

            # fix overlapping undercut
            utils.subtract_overlap_from_layer(self.resistor.device, [self.top_connector, self.bot_connector], res_params.undercut_layer, res_params.undercut_spacing)

            self.shunt_resistor = self.device << self.resistor.device

            utils.add_writefield(writefield_params, self.device)

            self.res_params = res_params
            self.squid_params = squid_params
            self.capa_params = capa_params
            self.writefield_params = writefield_params


    