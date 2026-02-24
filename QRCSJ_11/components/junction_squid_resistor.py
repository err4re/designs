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
from components.resistor import Resistor, ResParams
from components.junction_resistor import JJResistor, CapaParams


class JJSquidResistor():

    def __init__(self) -> None:
        self.jj_params: Optional[JJParams] = None
        self.res_params: Optional[ResParams] = None
        self.squid_params: Optional[SquidParams] = None
        self.capa_params: Optional[CapaParams] = None
        self.writefield_params: Optional[WritefieldParams] = None

        self.device: Optional[Device] = None

        self.junction: Optional[JJ] = None
        self.squid: Optional[Squid] = None
        self.resistor: Optional[Resistor] = None

        self.jj_ref: Optional[DeviceReference] = None
        self.squid_ref: Optional[DeviceReference] = None
        self.shunt_resistor: Optional[DeviceReference] = None

        self.top_connector: Optional[DeviceReference] = None
        self.bot_connector: Optional[DeviceReference] = None

    @staticmethod
    def sync_parameters(jj_params: JJParams, squid_params: SquidParams, res_params: ResParams, capa_params: CapaParams) -> Tuple[ResParams, JJParams, CapaParams]:
        jj_params = replace(jj_params)

        if res_params.small_resistor:
            jj_params.total_length = (res_params.num_segments)* res_params.spacing + res_params.num_segments*res_params.resistor_width + res_params.connector_height+2*capa_params.length_y+squid_params.total_length
        else:
            jj_params.total_length = res_params.total_spacing+(res_params.num_segments-2)*res_params.resistor_width+2*res_params.connector_height +2*capa_params.length_y+squid_params.total_length

        capa_params = replace(capa_params)
        utils.sync_attributes(res_params, capa_params)
        capa_params.arm_width = jj_params.arm_width

        return jj_params, squid_params, res_params, capa_params
    
    def add_ports(self) -> None:
        self.device.add_port(name= 'capa top', port=self.top_connector.ports['capa out'])
        self.device.add_port(name= 'capa bot', port=self.bot_connector.ports['capa out'])



    def generate_junction_squid_resistor(self, jj_params: JJParams, squid_params: SquidParams, res_params: ResParams, capa_params: CapaParams, writefield_params: WritefieldParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('JJ with Squid and shunt resistor already exists.')
        else:
            self.device = Device('JJ with squid and resistive shunt')

            # synch parameters, derive parameters from resistor parameters
            jj_params, squid_params, res_params, capa_params = JJSquidResistor.sync_parameters(jj_params, squid_params, res_params, capa_params)

            self.resistor = Resistor()
            self.resistor.generate_resistor(res_params)
            self.shunt_resistor = self.device << self.resistor.device

            self.jj = JJ()
            self.jj.generate_jj(jj_params)
            self.jj_ref = self.device << self.jj.device

            self.squid = Squid()
            self.squid.generate_squid(squid_params)
            self.squid_ref = self.device << self.squid.device

            Connector = JJResistor.create_connector(capa_params)

            # fix overlapping undercut
            Connector.move(origin=Connector.ports['in'], destination=self.jj_ref.ports['top'])
            utils.subtract_overlap_from_layer(Connector, self.jj_ref, res_params.undercut_layer, res_params.undercut_spacing)

            self.top_connector = self.device << Connector        
            self.bot_connector = self.device << Connector

            # put bot connector in the right place
            self.bot_connector.mirror(p1=(-1,0), p2=(1,0))

            self.squid_ref.connect('top', self.top_connector.ports['out'])
            
            if res_params.small_resistor:
                self.shunt_resistor.move(self.shunt_resistor.ports['top left small'], self.squid_ref.ports['bot'])
                self.shunt_resistor.movex(res_params.arm_width/2 - squid_params.arm_width/2)
            else:
                self.shunt_resistor.move(self.shunt_resistor.ports['top'], self.squid_ref.ports['bot'])
                self.shunt_resistor.move((-squid_params.arm_width/2, -res_params.connector_height/2))

            self.add_ports()

            # fix overlapping undercut
            utils.subtract_overlap_from_layer(self.device, [self.top_connector, self.bot_connector, self.squid_ref], res_params.undercut_layer, res_params.undercut_spacing)


            # # move resistor to final position
            # if res_params.small_resistor and not res_params.connectors:
            #     self.resistor.device.move(origin=self.resistor.device.ports['top left small'], destination=self.top_connector.ports['out'])
            #     self.resistor.device.movex(res_params.arm_width/2 - capa_params.arm_width/2)

            # else:
            #     self.resistor.device.move(origin=self.resistor.device.ports['top'], destination=self.top_connector.ports['out'])
            #     self.resistor.device.move((-capa_params.arm_width/2,-res_params.connector_height/2))

            
            utils.add_writefield(writefield_params, self.device)

            self.res_params = res_params
            self.squid_params = squid_params
            self.capa_params = capa_params
            self.writefield_params = writefield_params