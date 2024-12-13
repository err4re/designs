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
from components.resistor import Resistor, ResParams

### define parameters for coupling capacitor
@dataclass
class CapaParams:
    length_x: float = 20
    length_y: float = 8

    connector_height: float = 0.8
    arm_width: float = 0.22

    undercut_width: float = 0.8
    undercut_spacing: float = 0.5

    no_resistor: bool = False

    ebeam_capa: bool = True
    ebeam_capa_spacing: float = 1.8
    ebeam_capa_height: float = 4

    ebeam_layer: Layer = default_ls['ebeam']
    undercut_layer: Layer = default_ls['undercut']

class JJResistor():
    def __init__(self) -> None:
        self.res_params: Optional[ResParams] = None
        self.jj_params: Optional[JJParams] = None
        self.capa_params: Optional[CapaParams] = None
        self.writefield_params: Optional[WritefieldParams] = None

        self.device: Optional[Device] = None

        self.jj: Optional[JJ] = None
        self.resistor: Optional[Resistor] = None

        self.jj_ref: Optional[DeviceReference] = None
        self.resistor_ref: Optional[DeviceReference] = None

        self.top_connector: Optional[Device] = None
        self.bot_connector: Optional[Device] = None

        self.device: Optional[Device] = None


    @staticmethod
    def sync_parameters(res_params: ResParams, jj_params: JJParams, capa_params: CapaParams) -> Tuple[ResParams, JJParams, CapaParams]:
        jj_params = replace(jj_params)

        if res_params.small_resistor:
            jj_params.total_length = res_params.total_spacing+(res_params.num_segments)*res_params.resistor_width+2*capa_params.length_y + res_params.spacing + res_params.connector_height
        elif res_params.connectors:
            jj_params.total_length = res_params.total_spacing+(res_params.num_segments-2)*res_params.resistor_width+2*res_params.connector_height+2*capa_params.length_y
        else:
            jj_params.total_length = res_params.total_spacing+(res_params.num_segments)*res_params.resistor_width+2*capa_params.length_y

        capa_params = replace(capa_params)
        utils.sync_attributes(res_params, capa_params)
        capa_params.arm_width = jj_params.arm_width

        return res_params, jj_params, capa_params


    @staticmethod
    def create_connector(capa_params:CapaParams) -> Device:
        Connector = Device('Connector')
        Horizontal, Undercut = Resistor.create_connector(ResParams(connector_width=capa_params.length_x, connector_height=capa_params.connector_height))

        if not capa_params.no_resistor:
            Vertical = pg.rectangle(size=(-capa_params.arm_width, -(capa_params.length_y)), layer=default_ls['ebeam'])
        
        try:
            Undercut = pg.boolean(A=Undercut, B=pg.offset(Vertical, distance=capa_params.undercut_spacing), operation='A-B', layer=capa_params.undercut_layer)
        except:
            print('failed offsetting undercut')

        if not capa_params.no_resistor:
            Connector << [Vertical, Horizontal, Undercut]
        else:
            Connector << [Horizontal, Undercut]

        Connector.add_port(name='in', midpoint=(-(capa_params.length_x-capa_params.arm_width/2), 0), width=capa_params.arm_width, orientation=-90)
        Connector.add_port(name='out', midpoint=(-capa_params.arm_width/2, -capa_params.length_y), width=capa_params.arm_width, orientation=-90)

        

        if capa_params.ebeam_capa:
            Horizontal, Undercut = Resistor.create_connector(ResParams(connector_width=capa_params.length_x, connector_height=capa_params.ebeam_capa_height))

            Ebeam_Capa = Horizontal + Undercut
            Ebeam_Capa.movey(capa_params.ebeam_capa_spacing + capa_params.connector_height)

            Connector << [Horizontal]

            Connector.add_port(name='capa out', midpoint=Horizontal.center, width=capa_params.length_x, orientation=90)
        else:
            Connector.add_port(name='capa out', midpoint=Horizontal.center + (0, capa_params.connector_height/2), width=capa_params.length_x, orientation=90)


        return Connector

    def add_ports(self) -> None:
            self.device.add_port(name= 'capa top', port=self.top_connector.ports['capa out'])
            self.device.add_port(name= 'capa bot', port=self.bot_connector.ports['capa out'])



    def generate_junction_resistor(self, res_params: ResParams, jj_params: JJParams, capa_params: CapaParams, writefield_params: WritefieldParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Junction with shunt resistor already exists.')
        else:
            self.device = Device('JJ with resistive shunt')

            # synch parameters, derive parameters from resistor parameters
            res_params, jj_params, capa_params = JJResistor.sync_parameters(res_params, jj_params, capa_params)

            self.resistor = Resistor()
            self.resistor.generate_resistor(res_params)

            self.jj = JJ()
            self.jj.generate_jj(jj_params)
            self.jj_ref = self.device << self.jj.device

            Connector = JJResistor.create_connector(capa_params)

            # fix overlapping undercut
            Connector.move(origin=Connector.ports['in'], destination=self.jj_ref.ports['top'])
            utils.subtract_overlap_from_layer(Connector, self.jj_ref, res_params.undercut_layer, res_params.undercut_spacing)

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

            self.resistor_ref = self.device << self.resistor.device

            utils.add_writefield(writefield_params, self.device)

            self.res_params = res_params
            self.jj_params = jj_params
            self.capa_params = capa_params
            self.writefield_params = writefield_params








