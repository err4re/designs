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
from components.square_spiral import SquareSpiral, SquareSpiralParams


### define double square spiral class
class DoubleSquareSpiral:
    """
    Class to represent and generate a doubly wound square spiral centered around the origin.

    Attributes:
        device (Device): A PHIDL device object representing the double square spiral.
        spiral_params (SquareSpiralParams): Parameters used for spiral generation.
        
    """

    def __init__(self) -> None:
        self.device: Optional[Device] = None
        self.spiral_params: Optional[SquareSpiralParams] = None

        self.left_hand_spiral: Optional[SquareSpiral] = None
        self.right_hand_spiral: Optional[SquareSpiral] = None

    @staticmethod
    def create_half_spiral(spiral_params: SquareSpiralParams) -> SquareSpiral:
        # create parameters for a half spiral
        halfspiral_params = replace(spiral_params)

        halfspiral_params.inner_diameter = spiral_params.inner_diameter + spiral_params.turn_spacing_y + spiral_params.res_width
        halfspiral_params.turn_spacing_x = 2*spiral_params.turn_spacing_x + spiral_params.connector_width
        halfspiral_params.turn_spacing_y = 2*spiral_params.turn_spacing_y + spiral_params.res_width

        HalfSpiral = SquareSpiral()
        HalfSpiral.generate_square_spiral(halfspiral_params)

        return HalfSpiral


    @staticmethod
    def create_left_hand_half_spiral(spiral_params: SquareSpiralParams) -> SquareSpiral:
        HalfSpiral = DoubleSquareSpiral.create_half_spiral(spiral_params)

        # center spiral for double spiral
        HalfSpiral.device.move((-(spiral_params.turn_spacing_x + spiral_params.res_width)/2, -(spiral_params.turn_spacing_y + spiral_params.res_width)/2))

        return HalfSpiral
    
    @staticmethod
    def create_right_hand_half_spiral(spiral_params: SquareSpiralParams) -> SquareSpiral:
        right_hand_spiral_params = replace(spiral_params)
        right_hand_spiral_params.left_side_undercut = False       
        HalfSpiral = DoubleSquareSpiral.create_half_spiral(right_hand_spiral_params)

        # fix handedness and position
        HalfSpiral.device.mirror(p1=(1,0), p2=(-1,0))
        HalfSpiral.device.mirror(p1=(0,1), p2=(0,-1))
        HalfSpiral.device.move((-spiral_params.turn_spacing_x - spiral_params.connector_width, spiral_params.turn_spacing_y + spiral_params.res_width))

        # center spiral for double spiral
        HalfSpiral.device.move((-(spiral_params.turn_spacing_x + spiral_params.res_width)/2, -(spiral_params.turn_spacing_y + spiral_params.res_width)/2))

        return HalfSpiral
    
    @staticmethod
    def create_left_connector(spiral_params: SquareSpiralParams) -> Tuple[Device, Device]:
        Left_Connector = Device('Left connector')
        Left_Connector << pg.rectangle(size=(spiral_params.connector_width, spiral_params.inner_diameter/2 + 2*spiral_params.res_width - spiral_params.res_width/2 + 2*spiral_params.connector_overlap), layer=spiral_params.layer)
        #undercut
        Undercut = Device('Undercut')
        Undercut << pg.rectangle(size=(spiral_params.connector_width, spiral_params.undercut_margin), layer=spiral_params.undercut_layer).movey(Left_Connector.ymax)
        Undercut << pg.rectangle(size=(spiral_params.connector_width, -spiral_params.undercut_margin), layer=spiral_params.undercut_layer)
    
        Connector = Left_Connector + Undercut
        Connector.move(origin=(Connector.xmin, Connector.ymax), 
                            destination=(-(spiral_params.inner_diameter)/2 - spiral_params.turn_spacing_y - spiral_params.res_width - spiral_params.connector_width,
                                        spiral_params.inner_diameter/2 + spiral_params.res_width + spiral_params.undercut_margin + spiral_params.connector_overlap))
        
        return Left_Connector, Undercut
    
    @staticmethod
    def create_vertical_connector(spiral_params: SquareSpiralParams) -> Tuple[Device, Device]:
        VerticalConnector = Device('Vertical connector')

        Vertical_Connector = pg.rectangle(size=(spiral_params.inner_diameter + 2*spiral_params.connector_overlap - spiral_params.res_width, spiral_params.res_width), layer=spiral_params.highdose_layer)
        #undercut
        Undercut = Device('Undercut')
        Undercut << pg.rectangle(size=(-spiral_params.undercut_margin, spiral_params.connector_width), layer=spiral_params.undercut_layer).movey(-spiral_params.res_width/2)
        
        Connector = Vertical_Connector + Undercut
        Connector.move((spiral_params.connector_width-(spiral_params.inner_diameter)/2 - spiral_params.turn_spacing_y - spiral_params.res_width - spiral_params.connector_width - spiral_params.connector_overlap, -spiral_params.res_width/2))
        
        return Vertical_Connector, Undercut
    
    @staticmethod
    def create_right_connector(spiral_params: SquareSpiralParams) -> Tuple[Device, Device]:
        Right_Connector = Device('Right connector')
        Right_Connector << pg.rectangle(size=(spiral_params.connector_width, spiral_params.inner_diameter/2 + 2*spiral_params.res_width - spiral_params.res_width/2 + 2*spiral_params.connector_overlap)
                                          , layer=spiral_params.layer)
        #undercut
        Undercut = Device('Undercut')
        Undercut << pg.rectangle(size=(spiral_params.connector_width, spiral_params.undercut_margin), layer=spiral_params.undercut_layer).movey(Right_Connector.ymax)
        Undercut << pg.rectangle(size=(spiral_params.connector_width, -spiral_params.undercut_margin), layer=spiral_params.undercut_layer)

        Connector = Right_Connector + Undercut
        Connector.move(origin=(Connector.xmin, Connector.ymax-spiral_params.undercut_margin),
                                destination=(spiral_params.inner_diameter + 2*spiral_params.connector_overlap - spiral_params.res_width + spiral_params.connector_width-(spiral_params.inner_diameter)/2 - spiral_params.turn_spacing_y - spiral_params.res_width - spiral_params.connector_width - spiral_params.connector_overlap - spiral_params.connector_overlap, spiral_params.connector_width/2 + spiral_params.connector_overlap))
        
        return Right_Connector, Undercut
        

    def create_central_connector(self, spiral_params: SquareSpiralParams) -> Device:
        CentralConnector = Device('Central connector')

        LeftConnector, LeftUndercut = DoubleSquareSpiral.create_left_connector(spiral_params)
        VerticalConnector, VerticalUndercut = DoubleSquareSpiral.create_vertical_connector(spiral_params)
        RightConnector, RightUndercut = DoubleSquareSpiral.create_right_connector(spiral_params)

        # cut outs to avoid overlaps with other structures
        LeftConnector = pg.boolean(A=LeftConnector, B=self.device, operation='A-B', layer=spiral_params.layer)
        RightConnector = pg.boolean(A=RightConnector, B=self.device, operation='A-B', layer=spiral_params.layer)

        VerticalConnector = pg.boolean(A=VerticalConnector, B=[LeftConnector, RightConnector], operation='A-B', layer=spiral_params.highdose_layer)
        
        CentralConnector << [LeftConnector, LeftUndercut, RightConnector, RightUndercut, VerticalConnector, VerticalUndercut]

        return CentralConnector
    
    def add_ports(self, spiral_params: SquareSpiralParams) -> Device:
        self.device.add_port(name='top', port=self.left_hand_spiral.device.ports['in'])
        self.device.add_port(name='bot', port=self.right_hand_spiral.device.ports['in'])

    

    def generate_double_square_spiral(self, spiral_params: SquareSpiralParams, overwrite: bool = False) -> Device:
        if (self.device is not None) and not overwrite:
            print('Spiral already exists.')
        else:
            self.device = Device('Double square spiral')

            # create the two spirals

            self.left_hand_spiral = DoubleSquareSpiral.create_left_hand_half_spiral(spiral_params)
            self.right_hand_spiral = DoubleSquareSpiral.create_right_hand_half_spiral(spiral_params)

            self.device << [self.left_hand_spiral.device, self.right_hand_spiral.device]

            self.device << self.create_central_connector(spiral_params)

            self.add_ports(spiral_params)

            self.spiral_params = spiral_params
            




