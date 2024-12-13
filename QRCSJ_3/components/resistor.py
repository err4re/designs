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
class ResParams:
    resistor_width: float = 0.11
    segment_length: float = 50
    adaptive_spacing: bool = False
    total_spacing: float = 68.2
    spacing: float = 3
    arm_height: float = 2.22 #should be spacing + 2*res_width
    arm_bottom: float = 0.65
    arm_width: float = 0.7
    num_segments: int = 32

    connector_width: float = 2
    connector_height: float = 1.2
    connectors: bool = True

    undercut_width: float = 0.8
    undercut_spacing: float = 0.06

    small_resistor: bool = False

    ebeam_layer: Layer = default_ls['ebeam']
    undercut_layer: Layer = default_ls['undercut']
    highdose_layer: Layer = default_ls['ebeam_high']

    def __post_init__(self):
        if self.adaptive_spacing:
            self.spacing = self.total_spacing/(self.num_segments-1)
            self.arm_height = self.spacing + 2*self.resistor_width
        else:
            self.arm_height = self.spacing + 2*self.resistor_width
            self.total_spacing = (self.num_segments-1)*self.spacing



### define resistor class
class Resistor:
    """
    Class to represent and generate a resistor centered around the origin.

    Attributes:
        device (Device): A PHIDL device object representing the resistor.
        res_params (ResParams): Parameters used for resistor generation.
        
    """

    def __init__(self) -> None:
        self.res_params: Optional[ResParams] = None

        self.device: Optional[Device] = None

        self.res_segments: Optional[Device] = None
        self.res_arms: Optional[Device] = None

        self.top_connector: Optional[DeviceReference] = None
        self.bot_connector: Optional[DeviceReference] = None

        self.liftoff_undercut: Optional[Device] = None

    @staticmethod
    def create_res_segment(res_params: ResParams) -> Device:
        Segment = Device('Resistive segment')
        Segment << pg.rectangle(size=(res_params.segment_length, res_params.resistor_width), layer=res_params.highdose_layer)

        return Segment


    @staticmethod
    def create_res_segments(res_params: ResParams) -> list[Device]:
        Segments = [Resistor.create_res_segment(res_params) for i in range(res_params.num_segments)]

        # move segments to their positions
        for i, Segment in enumerate(Segments):
            Segment.move((0,  - i*(res_params.arm_height- res_params.resistor_width)))

        return Segments
    
    @staticmethod
    def create_arm(res_params: ResParams) -> Tuple[Device, Device]:
        Arm = Device('Arm connector')

        # for a small resistor the connector puts the resistive segments in parallel
        if res_params.small_resistor:
            height = res_params.total_spacing + res_params.num_segments*res_params.resistor_width
        # for big resistors the resistive segments are in series
        else:
            height = res_params.arm_height
        
        Arm << pg.rectangle(size=(res_params.arm_width, height), layer=res_params.ebeam_layer)
       
        Undercut_Box = pg.rectangle(size=(res_params.arm_width + res_params.undercut_width, height + 2*res_params.undercut_width + res_params.arm_bottom), layer=res_params.undercut_layer)
        Undercut_Box.move((-res_params.undercut_width, -res_params.undercut_width - res_params.arm_bottom))
        
        Arm << pg.rectangle(size=(res_params.arm_width, -res_params.arm_bottom), layer=res_params.ebeam_layer)

        Undercut = Device('Undercut')
        Undercut << pg.boolean(A=Undercut_Box, B=pg.offset(Arm, distance=res_params.undercut_spacing), operation='A-B', layer=res_params.undercut_layer)

        return Arm, Undercut

   
    def create_left_arms(self, res_params: ResParams) -> Tuple[list[Device], list[Device]]:
        Arms = []
        Undercuts = []

        # small resistor has only one arm on each side, all resistive segments in parallel
        if res_params.small_resistor:
            Arm, Undercut = Resistor.create_arm(res_params)

            Arm.move((- res_params.arm_width, - (res_params.total_spacing + (res_params.num_segments-1)*res_params.resistor_width)))
            Undercut.move((- res_params.arm_width,  - (res_params.total_spacing + (res_params.num_segments-1)*res_params.resistor_width)))

            Arms.append(Arm)
            Undercuts.append(Undercut)
        # for big resistors all resistive segments are in series
        else:
            for i in range(res_params.num_segments // 2 - 1 + res_params.num_segments%2):
                Arm, Undercut = Resistor.create_arm(res_params)

                Arm.move((- res_params.arm_width, -res_params.arm_height - (2*i+1)*(res_params.arm_height - res_params.resistor_width) + res_params.resistor_width))
                Undercut.move((- res_params.arm_width, -res_params.arm_height - (2*i+1)*(res_params.arm_height - res_params.resistor_width) + res_params.resistor_width))

                Arms.append(Arm)
                Undercuts.append(Undercut)

        return Arms, Undercuts
    
    @staticmethod
    def create_right_arm_small(res_params: ResParams) -> Tuple[Device, Device]:
        Arm = Device('Right arm connector')

        Lr = Arm << pg.L(width=res_params.connector_height, 
                         size=(res_params.segment_length + res_params.connector_height/2 + res_params.arm_width,
                              res_params.total_spacing + res_params.spacing + (res_params.num_segments)*res_params.resistor_width + res_params.connector_height/2),
                         layer=res_params.ebeam_layer)

        Undercut = Device('Undercut')
        Undercut_Box =  pg.offset(Arm, distance=res_params.undercut_width/2, layer=res_params.undercut_layer)
        Undercut_Box.move((res_params.undercut_width/2, res_params.undercut_width/2))
        Undercut_Box << pg.offset(Arm, distance=res_params.undercut_width/2, layer=res_params.undercut_layer).move((res_params.undercut_width/2, -res_params.undercut_width/2))

        Undercut << pg.boolean(Undercut_Box, pg.offset(Arm, res_params.undercut_spacing), operation='A-B', layer=res_params.undercut_layer) 

        Right_Arm = Arm + Undercut
        Right_Arm.mirror(p1=(0,1), p2=(0,-1))
        Right_Arm.move((-res_params.arm_width/2, res_params.arm_width/2))


        return Arm, Undercut
    
 
    def create_right_arms(self, res_params: ResParams) -> Tuple[list[Device], list[Device]]:
        Arms = []
        Undercuts = []

        # small resistor has only one arm on each side, all resistive segments in parallel
        if res_params.small_resistor:
            Arm, Undercut = Resistor.create_right_arm_small(res_params)
            Right_Arm = Arm + Undercut

            Right_Arm.move(origin=(Arm.xmax - res_params.connector_height, Arm.ymax), destination=(self.res_segments.xmax, self.res_segments.ymax))
            
            # subtract resistive segements from undercuts to avoid overlap
            Undercut = pg.boolean(A=Undercut, B=pg.offset(elements=self.res_segments, distance=res_params.undercut_spacing), operation='A-B', layer=res_params.undercut_layer)


            Arms.append(Arm)
            Undercuts.append(Undercut)
        # for big resistors all resistive segments are in series
        else:
            for i in range(res_params.num_segments // 2):
                Arm, Undercut = Resistor.create_arm(res_params)

                Arm.move((res_params.segment_length, -res_params.arm_height + res_params.resistor_width  - 2*i*(res_params.arm_height - res_params.resistor_width)))
                Undercut.move((res_params.segment_length, -res_params.arm_height + res_params.resistor_width  - 2*i*(res_params.arm_height - res_params.resistor_width)))

                # subtract resistive segements from undercuts to avoid overlap
                Undercut = pg.boolean(A=Undercut, B=pg.offset(elements=self.res_segments, distance=res_params.undercut_spacing), operation='A-B', layer=res_params.undercut_layer)


                Arms.append(Arm)
                Undercuts.append(Undercut)

        return Arms, Undercuts
    
    def create_arms(self, res_params: ResParams) -> Tuple[list[Device], list[Device]]:
        Arms = []
        Undercuts = []

        left_arms, undercuts = self.create_left_arms(res_params)
        Arms.append(left_arms)
        Undercuts.append(undercuts)

        right_arms, undercuts = self.create_right_arms(res_params)
        Arms.append(right_arms)
        Undercuts.append(undercuts)

        return Arms, Undercuts

    
    @staticmethod
    def create_connector(res_params: ResParams) -> Tuple[Device, Device]:
        Connector = Device('Connector')

        Connector << pg.rectangle(size=(-res_params.connector_width, res_params.connector_height), layer=res_params.ebeam_layer)
        
        Undercut_Box = pg.rectangle(size=(-res_params.connector_width - res_params.undercut_width, res_params.connector_height +2*res_params.undercut_width), layer=res_params.undercut_layer)
        Undercut_Box.movey(-res_params.undercut_width)

        Undercut = Device('Undercut')
        Undercut << pg.boolean(A=Undercut_Box, B=pg.offset(Connector, distance=res_params.undercut_spacing), operation='A-B', layer=res_params.undercut_layer)

        return Connector, Undercut
    
 
    def create_connectors(self, res_params: ResParams) -> Tuple[Device, Device, Device]:

        Top_Connector = Device('Top connector')
        Bot_Connector = Device('Bot connector')
        Undercut = Device('Undercut')

        connector, undercut = Resistor.create_connector(res_params)
        Top_Connector << connector
        Undercut << undercut


        # odd number of resistive segments?
        if res_params.num_segments%2:
            connector, undercut = Resistor.create_connector(res_params)
            # make group to manipulate devices together
            bot_connector = connector + undercut
            if res_params.small_resistor:
                bot_connector.move(origin=(bot_connector.xmax, bot_connector.y), destination=(self.res_arms.xmin+res_params.arm_width, self.res_arms.ymin+res_params.connector_height/2))
            else:
                bot_connector.move(origin=(connector.xmin, connector.ymax), destination=(self.res_segments.xmax, self.res_segments.ymin + res_params.resistor_width))

            # subtract resistive segements from undercuts to avoid overlap
            undercut = pg.boolean(A=undercut, B=pg.offset(elements=self.res_segments, distance=res_params.undercut_spacing), operation='A-B', layer=res_params.undercut_layer)

            Bot_Connector << connector
            Undercut << undercut

        # even number of resistive segments
        else:
            connector, undercut = Resistor.create_connector(res_params)
            # make group to manipulate devices together
            bot_connector = connector + undercut
            if res_params.small_resistor:
                bot_connector.move(origin=(bot_connector.xmax, bot_connector.y), destination=(self.res_arms.xmin+res_params.arm_width, self.res_arms.ymin+res_params.connector_height/2))
            else:    
                bot_connector.move(origin=(connector.xmax, connector.ymax), destination=(self.res_segments.xmin, self.res_segments.ymin + res_params.resistor_width))

            Bot_Connector << connector
            Undercut << undercut
            
        return Top_Connector, Bot_Connector, Undercut
    
    def add_ports(self, res_params: ResParams) -> None:

        if res_params.connectors:
            self.device.add_port(name='top', midpoint=(self.top_connector.xmin, (self.top_connector.ymax + self.top_connector.ymin)/2), width=res_params.connector_height, orientation=180)
        self.device.add_port(name='top right', midpoint=(self.res_arms.xmax, self.res_arms.ymax - res_params.arm_height/2), width=res_params.arm_height, orientation=0)

        if res_params.small_resistor:
            self.device.add_port(name='top left small', midpoint=(self.res_arms.xmin+res_params.arm_width/2, self.res_arms.ymax), width=res_params.arm_width, orientation=90)
            self.device.add_port(name='bot left small', midpoint=(self.res_arms.xmin+res_params.arm_width/2, self.res_arms.ymin), width=res_params.arm_width, orientation=-90)
            
            if res_params.connectors:
                self.device.add_port(name='bot', midpoint=(self.bot_connector.xmin, (self.bot_connector.ymax + self.bot_connector.ymin)/2), width=res_params.connector_height, orientation=180)


        else:
            # odd number of resistive segments?
            if res_params.num_segments%2:
                if res_params.connectors:
                    self.device.add_port(name='bot', midpoint=(self.bot_connector.xmax, (self.bot_connector.ymax + self.bot_connector.ymin)/2), width=res_params.connector_height, orientation=0)

                self.device.add_port(name='bot left', midpoint=(self.res_arms.xmin, self.res_arms.ymin + res_params.arm_height/2), width=res_params.arm_height, orientation=180)

            # even number of resistive segments
            else:
                if res_params.connectors:
                    self.device.add_port(name='bot', midpoint=(self.bot_connector.xmin, (self.bot_connector.ymax + self.bot_connector.ymin)/2), width=res_params.connector_height, orientation=180*((res_params.num_segments+1)%2))

                self.device.add_port(name='bot right', midpoint=(self.res_arms.xmax, self.res_arms.ymin + res_params.arm_height/2), width=res_params.arm_height, orientation=0)

        
                
    
    def generate_resistor(self, res_params:ResParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Resistor already exists.')
        else:
            self.device = Device('Resistor')
            self.res_arms = Device('Arms')
            self.res_segments = Device('Resistive segments')
            self.liftoff_undercut = Device('Liftoff undercut')

            self.device << [self.res_arms, self.res_segments, self.liftoff_undercut]

            self.res_segments << self.create_res_segments(res_params)

            res_arms, undercuts = self.create_arms(res_params)
            self.res_arms << res_arms
            self.liftoff_undercut << undercuts
            
            if res_params.connectors:
                top_connector, bot_connector, undercut = self.create_connectors(res_params)

                self.top_connector = self.device << top_connector
                self.bot_connector = self.device << bot_connector

                self.liftoff_undercut << undercut

            self.add_ports(res_params)

            if res_params.small_resistor and res_params.connectors:
                utils.unify_layer(self.device, res_params.undercut_layer)
                utils.unify_layer(self.device, res_params.ebeam_layer)
                utils.subtract_layers(self.device, layer_a=res_params.undercut_layer, layer_b=res_params.ebeam_layer, offset=res_params.undercut_spacing)


            self.res_params = res_params


