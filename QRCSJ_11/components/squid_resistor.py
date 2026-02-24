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

from copy import deepcopy


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
        capa_params = replace(capa_params)

        # utils.sync_attributes(res_params, capa_params)
        capa_params.arm_width = squid_params.arm_width

        if res_params.small_resistor:
            squid_params.total_length = res_params.total_spacing + max(res_params.min_spacing, res_params.spacing) + (res_params.num_segments)*res_params.resistor_width + res_params.connector_height + 2*capa_params.length_y
        elif res_params.connectors:
            squid_params.total_length = res_params.total_spacing+(res_params.num_segments-2)*res_params.resistor_width+2*res_params.connector_height+2*capa_params.length_y
        else:
            squid_params.total_length = res_params.total_spacing+(res_params.num_segments)*res_params.resistor_width+2*capa_params.length_y

        # make sure the squid loop is smaller than the total length
        if squid_params.total_length < squid_params.loop_height:
            squid_params.total_length = squid_params.loop_height + 2*squid_params.island_height + 2*capa_params.length_y



        # check for fixed device height and adjust y length of connector
        if capa_params.fixed_device_height:
            
            if res_params.small_resistor:
                delta_capa_length_y = (capa_params.device_height - squid_params.total_length )/2 - capa_params.connector_height 
                capa_params.length_y += delta_capa_length_y
                squid_params.total_length += 2*delta_capa_length_y
            elif res_params.connectors:
                delta_capa_length_y = (capa_params.device_height - squid_params.total_length )/2 - capa_params.connector_height 
                capa_params.length_y += (delta_capa_length_y)
                squid_params.total_length += 2*delta_capa_length_y


        # check for mirrored x-axis in resistor
        capa_params.mirrored_x_axis = res_params.mirrored_x_axis
        

        return res_params, squid_params, capa_params
    
    def add_ports(self, capa_params: CapaParams) -> None:

        if capa_params.ebeam_capa_xl30:
            self.device.add_port(name= 'capa top', port=self.top_connector.ports['xl30 top capa'])
            self.device.add_port(name= 'capa bot', port=self.bot_connector.ports['xl30 bot capa'])

        else:
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

         
            if capa_params.ebeam_capa_xl30:

                if res_params.mirrored_x_axis:
                    # Top_Connector = JJResistor.create_connector(capa_params)
                    # Top_Connector.add_port(name='xl30 top capa', midpoint=Top_Connector.ports['capa out'].center, orientation=0, width=capa_params.ebeam_capa_xl30_height)

                    # # fix overlapping undercut
                    # Top_Connector.move(origin=Top_Connector.ports['in'], destination=self.squid_ref.ports['top'])
                    # utils.subtract_overlap_from_layer(Top_Connector, self.squid_ref, res_params.undercut_layer, res_params.undercut_spacing)

                    # self.top_connector = self.device << Top_Connector


                    capa_params_top = deepcopy(capa_params)
                    capa_params_top.ebeam_capa_xl30_height = capa_params.ebeam_capa_xl30_finger_height
                    Top_Connector = JJResistor.create_connector(capa_params_top)

                    # fix overlapping undercut
                    Top_Connector.move(origin=Top_Connector.ports['in'], destination=self.squid_ref.ports['top'])
                    utils.subtract_overlap_from_layer(Top_Connector, self.squid_ref, res_params.undercut_layer, res_params.undercut_spacing)

                    Box, Undercut = Resistor.create_connector(ResParams(connector_width=capa_params.ebeam_capa_xl30_width, connector_height=capa_params.ebeam_capa_xl30_height, ebeam_layer=capa_params.ebeam_capa_layer, undercut_layer=capa_params.undercut_layer))
                    Top_Overlap_Box = Device('Bot Overlap Box')
                    Top_Overlap_Box << [Box, Undercut]
                    Top_Overlap_Box.add_port(name='xl30 top capa', midpoint=Box.center, orientation=0, width=capa_params.ebeam_capa_xl30_height)

                    Box, Undercut = Resistor.create_connector(ResParams(connector_width=capa_params.ebeam_capa_xl30_finger_width - capa_params.ebeam_capa_width, connector_height=capa_params.ebeam_capa_xl30_finger_height, ebeam_layer=capa_params.ebeam_capa_layer, undercut_layer=capa_params.undercut_layer))
                    Finger = Box + Undercut
                    Finger.movex(-capa_params.ebeam_capa_xl30_width)
                    Finger.movey(Top_Overlap_Box.ymax - Finger.ymax)
                    Top_Overlap_Box << [Box, Undercut]

                    Top_Overlap_Box.movex(capa_params.ebeam_capa_xl30_finger_width + capa_params.ebeam_capa_xl30_width - squid_params.arm_width/2)
                    Top_Overlap_Box.movey(Top_Connector.ymax - Top_Overlap_Box.ymax)

                    Top_Connector << Top_Overlap_Box
                    Top_Connector.add_port(port=Top_Overlap_Box.ports['xl30 top capa'])

                    self.top_connector = self.device << Top_Connector


                    Bot_Connector = JJResistor.create_connector(capa_params)
                    Bot_Connector.add_port(name='xl30 bot capa', midpoint=Bot_Connector.ports['capa out'].center, orientation=180, width=capa_params.ebeam_capa_xl30_height)

                    # fix overlapping undercut
                    Bot_Connector.move(origin=Bot_Connector.ports['in'], destination=self.squid_ref.ports['top'])
                    utils.subtract_overlap_from_layer(Bot_Connector, self.squid_ref, res_params.undercut_layer, res_params.undercut_spacing)

                    self.bot_connector = self.device << Bot_Connector

                    # put bot connector in the right place
                    self.bot_connector.mirror(p1=(-1,0), p2=(1,0))

                else:
                    Top_Connector = JJResistor.create_connector(capa_params)
                    Top_Connector.add_port(name='xl30 top capa', midpoint=Top_Connector.ports['capa out'].center, orientation=0, width=capa_params.ebeam_capa_xl30_height)

                    # fix overlapping undercut
                    Top_Connector.move(origin=Top_Connector.ports['in'], destination=self.squid_ref.ports['top'])
                    utils.subtract_overlap_from_layer(Top_Connector, self.squid_ref, res_params.undercut_layer, res_params.undercut_spacing)

                    self.top_connector = self.device << Top_Connector


                    capa_params_bot = deepcopy(capa_params)
                    capa_params_bot.ebeam_capa_xl30_width = capa_params.ebeam_capa_xl30_finger_width
                    capa_params_bot.ebeam_capa_xl30_height = capa_params.ebeam_capa_xl30_finger_height
                    Bot_Connector = JJResistor.create_connector(capa_params_bot)

                    Box, Undercut = Resistor.create_connector(ResParams(connector_width=capa_params.ebeam_capa_xl30_width, connector_height=capa_params.ebeam_capa_xl30_height, ebeam_layer=capa_params.ebeam_capa_layer, undercut_layer=capa_params.undercut_layer))
                    Bot_Overlap_Box = Device('Bot Overlap Box')
                    Bot_Overlap_Box << [Box, Undercut]
                    Bot_Overlap_Box.add_port(name='xl30 bot capa', midpoint=Box.center, orientation=180, width=capa_params.ebeam_capa_xl30_height)

                    Bot_Overlap_Box.movex(-capa_params.ebeam_capa_xl30_finger_width)
                    Bot_Overlap_Box.movey(Bot_Connector.ymax - Bot_Overlap_Box.ymax)

                    Bot_Connector << Bot_Overlap_Box
                    Bot_Connector.add_port(port=Bot_Overlap_Box.ports['xl30 bot capa'])



                    # fix overlapping undercut
                    Bot_Connector.move(origin=Bot_Connector.ports['in'], destination=self.squid_ref.ports['top'])
                    utils.subtract_overlap_from_layer(Bot_Connector, self.squid_ref, res_params.undercut_layer, res_params.undercut_spacing)

                    self.bot_connector = self.device << Bot_Connector

                    # put bot connector in the right place
                    self.bot_connector.mirror(p1=(-1,0), p2=(1,0))
                

            else:
                Connector = JJResistor.create_connector(capa_params)

                # fix overlapping undercut
                Connector.move(origin=Connector.ports['in'], destination=self.squid_ref.ports['top'])
                utils.subtract_overlap_from_layer(Connector, self.squid_ref, res_params.undercut_layer, res_params.undercut_spacing)

                self.top_connector = self.device << Connector        
                self.bot_connector = self.device << Connector

                # put bot connector in the right place
                self.bot_connector.mirror(p1=(-1,0), p2=(1,0))


            if res_params.mirrored_x_axis:
                self.top_connector.movex( - (capa_params.length_x - capa_params.arm_width))
                self.bot_connector.movex( - (capa_params.length_x - capa_params.arm_width))

            # move resistor to final position
            if res_params.small_resistor and not res_params.connectors:
                self.resistor.device.move(origin=self.resistor.device.ports['top left small'], destination=self.top_connector.ports['out'])
                self.resistor.device.movex(res_params.arm_width/2 - capa_params.arm_width/2)

                if res_params.mirrored_x_axis:
                    self.resistor.device.movex(-res_params.arm_width + capa_params.arm_width)

            else:
                self.resistor.device.move(origin=self.resistor.device.ports['top'], destination=self.top_connector.ports['out'])
                self.resistor.device.move((-capa_params.arm_width/2,-res_params.connector_height/2))

                if res_params.mirrored_x_axis:
                    self.resistor.device.movex(-res_params.connector_width + capa_params.arm_width)

            self.add_ports(capa_params)

            # fix overlapping undercut
            utils.subtract_overlap_from_layer(self.resistor.device, [self.top_connector, self.bot_connector], res_params.undercut_layer, res_params.undercut_spacing)
            self.shunt_resistor = self.device << self.resistor.device

            utils.subtract_overlap_from_layer(self.device, [self.top_connector, self.bot_connector, self.squid_ref], capa_params.undercut_layer, res_params.undercut_spacing)
        
            utils.add_writefield(writefield_params, self.device)

            self.res_params = res_params
            self.squid_params = squid_params
            self.capa_params = capa_params
            self.writefield_params = writefield_params


    