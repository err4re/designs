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
from components.junction import JJ, JJParams


### define parameters
@dataclass
class SquidParams(JJParams):
    """
    Parameters for generating a SQUID device, extending JJParams with additional SQUID-specific parameters.
    """
    total_length: float = 30

    bridge_width: float = 0.4
    jj_width: float = 0.2

    arm_width: float = 0.22

    island_width: float = 20.22
    island_height: float = 1.2
    loop_height: float = 15

    connector_width: float = 0.22
    connectors: bool = True

    x_spacing: float = 20

    y_spacing: float = 5

    undercut_width: float = 0.8
    undercut_spacing: float = 0.06

    layer: Layer = default_ls['ebeam']
    undercut_layer: Layer = default_ls['undercut']
    jj_undercut_layer: Layer = default_ls['jj_undercut']
    
    
    # Method to extract JJParams from a SquidParams instance
    def to_jj_params(self) -> JJParams:
        """
        Converts SQUID parameters to JJParams.
        
        Returns:
            JJParams: The junction parameters derived from SQUID parameters.
        """

        # Set total length of junction element
        self.jj_total_length = self.loop_height

        return JJParams(
            bridge_width=self.bridge_width,
            jj_width=self.jj_width,
            jj_length=self.jj_length,
            arm_width=self.arm_width,
            total_length=self.jj_total_length,
            undercut_width=self.undercut_width,
            layer=self.layer,
            undercut_layer=self.jj_undercut_layer,
            stress_boxes_width = self.stress_boxes_width,
            stress_boxes_height=self.stress_boxes_height,
            undercut_shape = self.undercut_shape,
            undercut_extension= self.undercut_extension,
            undercut_spacing_h=self.undercut_spacing_h,
            undercut_spacing_v=self.undercut_spacing_v
        )
    

### define Squid class
class Squid:
    """
    Class to represent and generate a Squid centered around the origin.

    Attributes:
        device (Device): A PHIDL device object representing the Squid.
        squid_params (SquidParams): Parameters used for Squid generation.
        left_jj (DeviceReference): Reference to the left Josephson junction.
        right_jj (DeviceReference): Reference to the right Josephson junction.
        left_junction (JJ): Left JJ object, containing design and configuration.
        right_junction (JJ): Right JJ object, containing design and configuration.
        top_island (DeviceReference): Reference to the top island of the SQUID.
        bot_island (DeviceReference): Reference to the bottom island of the SQUID.
        top_connector (DeviceReference): Reference to the top connector if specified.
        bot_connector (DeviceReference): Reference to the bottom connector if specified.
        liftoff_undercut (DeviceReference): Reference to the liftoff undercut device.
    """

    def __init__(self) -> None:
        self.squid_params: Optional[SquidParams] = None

        self.device: Optional[Device] = None

        self.left_jj: Optional[DeviceReference] = None
        self.right_jj: Optional[DeviceReference] = None

        self.left_junction: Optional[JJ] = None
        self.right_junction: Optional[JJ] = None

        self.top_island: Optional[DeviceReference] = None
        self.bot_island: Optional[DeviceReference] = None

        self.top_connector: Optional[DeviceReference] = None
        self.bot_connector: Optional[DeviceReference] = None

        self.liftoff_undercut: Optional[Device] = None

    ### methods to create islands
    @staticmethod
    def create_island(squid_params: SquidParams) -> Device:
        Island = Device('Island')

        Island << pg.rectangle(size=(squid_params.island_width+squid_params.arm_width, squid_params.island_height), layer=squid_params.layer)
        
        return Island

    @staticmethod
    def create_top_island(squid_params: SquidParams) -> Device:
        Top_Island = Device('Top island')

        Top_Island << Squid.create_island(squid_params)
        Top_Island.move((-squid_params.arm_width/2, squid_params.loop_height/2))

        return Top_Island

    @staticmethod
    def create_bot_island(squid_params: SquidParams) -> Device:
        Bot_Island = Device('Bot island')

        Bot_Island << Squid.create_island(squid_params)
        Bot_Island.move((-squid_params.arm_width/2, -(squid_params.loop_height/2 + squid_params.island_height)))

        return Bot_Island
    
    @staticmethod
    def create_connector(squid_params: SquidParams) -> Device:
        Connector = Device('Connector')
        
        Connector << pg.rectangle(size=(squid_params.connector_width, -(squid_params.total_length/2 - squid_params.loop_height/2 - squid_params.island_height)), layer=squid_params.layer)

        return Connector


    @staticmethod
    def create_top_connector(squid_params: SquidParams) -> Device:
        Top_Connector = Device('Top connector')
        
        Top_Connector << Squid.create_connector(squid_params)
        Top_Connector.move(( - squid_params.connector_width/2 + squid_params.island_width/2, squid_params.total_length/2))

        return Top_Connector
    
    @staticmethod
    def create_bot_connector(squid_params: SquidParams) -> Device:
        Bot_Connector = Device('Bot connector')

        Bot_Connector << Squid.create_connector(squid_params)
        Bot_Connector.mirror(p1=(-1,0), p2=(1,0))
        Bot_Connector.move((- squid_params.connector_width/2 + squid_params.island_width/2, - squid_params.total_length/2))

        return Bot_Connector
    
    def create_liftoff_undercut(self, squid_params: SquidParams) -> Device:
        if (self.device is None):
            print('Cannot create liftoff undercut, create device first.')
        else:
            Liftoff_Undercut = Device('Liftoff undercut')
            
            Top_Undercut = pg.rectangle(size=(squid_params.island_width+squid_params.arm_width + squid_params.undercut_width, squid_params.island_height + 2*squid_params.undercut_width), layer=squid_params.undercut_layer)
            Top_Undercut.move((-squid_params.arm_width/2 - squid_params.undercut_width, squid_params.loop_height/2 - squid_params.undercut_width))
        
            Bot_Undercut = pg.rectangle(size=(squid_params.island_width+squid_params.arm_width + squid_params.undercut_width, squid_params.island_height + 2*squid_params.undercut_width), layer=squid_params.undercut_layer)
            Bot_Undercut.move((-squid_params.arm_width/2 - squid_params.undercut_width, -(squid_params.loop_height/2 + squid_params.island_height) - squid_params.undercut_width))

            Liftoff_Undercut << pg.boolean(A=[Top_Undercut, Bot_Undercut], B=pg.offset(self.device, distance=squid_params.undercut_spacing), operation='A-B', layer=squid_params.undercut_layer) 

            return Liftoff_Undercut
        
    def add_ports(self, squid_params: SquidParams) -> None:
        if squid_params.connectors:
            self.device.add_port(name='top', midpoint=((self.top_island.xmax + self.top_island.xmin)/2, self.device.ymax), width=squid_params.connector_width, orientation=90)
            self.device.add_port(name='bot', midpoint=((self.bot_island.xmax + self.bot_island.xmin)/2, self.device.ymin), width=squid_params.connector_width, orientation=-90)
        else:
            self.device.add_port(name='top', midpoint=((self.top_island.xmax + self.top_island.xmin)/2, self.top_island.ymax), width=squid_params.connector_width, orientation=90)
            self.device.add_port(name='bot', midpoint=((self.bot_island.xmax + self.bot_island.xmin)/2, self.bot_island.ymin), width=squid_params.connector_width, orientation=-90)

        self.device.add_port(name='top left', midpoint=(self.top_island.xmin, (self.top_island.ymax + self.top_island.ymin)/2), width=squid_params.island_height, orientation=180)
        self.device.add_port(name='top right', midpoint=(self.top_island.xmax, (self.top_island.ymax + self.top_island.ymin)/2), width=squid_params.island_height, orientation=0)
        self.device.add_port(name='bot left', midpoint=(self.bot_island.xmin, (self.bot_island.ymax + self.bot_island.ymin)/2), width=squid_params.island_height, orientation=180)
        self.device.add_port(name='bot right', midpoint=(self.bot_island.xmax, (self.bot_island.ymax + self.bot_island.ymin)/2), width=squid_params.island_height, orientation=0)
        



    def generate_squid(self, squid_params: SquidParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Squid already exists.')
        else:
            self.device= Device('Squid')

            # create the two junctions and move them to their positions
            self.left_junction, self.right_junction = JJ(), JJ()

            self.left_junction.generate_jj(squid_params.to_jj_params())
            self.right_junction.generate_jj(squid_params.to_jj_params())

            self.left_jj, self.right_jj = self.device << [self.left_junction.device, self.right_junction.device]

            self.right_jj.movex(squid_params.island_width)

            # create islands
            self.top_island = self.device << self.create_top_island(squid_params)
            self.bot_island = self.device << self.create_bot_island(squid_params)

            # create connectors
            if squid_params.connectors:
                self.top_connector = self.device << self.create_top_connector(squid_params)
                self.bot_connector = self.device << self.create_bot_connector(squid_params)

            # create liftoff undercut
            self.liftoff_undercut = Device('Liftoff undercut')
            self.device << self.liftoff_undercut
            self.liftoff_undercut << self.create_liftoff_undercut(squid_params)

            # add ports
            self.add_ports(squid_params)

            # center squid
            self.device.move(origin=((self.top_island.xmax + self.top_island.xmin)/2, (self.top_island.ymax + self.bot_island.ymin)/2), destination=(0,0))

            self.device.remove(self.liftoff_undercut)

            self.squid_params = squid_params