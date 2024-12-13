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


### define parameters
@dataclass
class SquareSpiralParams():

    num_turns: int = 8
    res_width: float = 0.11
    connector_width: float = 0.2
    connector_overlap_x: float = 0.85
    connector_overlap_y: float = 0.7

    connector: bool = True
    connector_length: float = 6

    inner_diameter: float = 10
    turn_spacing: float = 2
    turn_spacing_x: float = 2
    turn_spacing_y: float = 2

    left_side_undercut: bool = True # undercut to the left on vertical segments? important for double square spiral
    undercut_margin: float = 0.8
    undercut_width: float = 0.3
    undercut_at_end: bool = True

    layer: Layer = default_ls['ebeam']
    undercut_layer: Layer = default_ls['jj_undercut']
    highdose_layer: Layer = default_ls['ebeam_high']


### define square spiral class
class SquareSpiral:
    """
    Class to represent and generate a square spiral centered around the origin.

    Attributes:
        device (Device): A PHIDL device object representing the square spiral.
        spiral_params (SquareSpiralParams): Parameters used for spiral generation.
        
    """

    def __init__(self) -> None:
        self.device: Optional[Device] = None
        self.spiral_params: Optional[SquareSpiralParams] = None

        self.res_segments: Optional[Device] = None
        self.conn_segments: Optional[Device] = None

        self.undercuts: Optional[Device] = None

    @staticmethod
    def create_square_spiral(spiral_params: SquareSpiralParams) -> Device:
        S = Device('Square spiral')

        ResSegments: list[Device] = []
        ConnSegments: list[Device] = []
        Undercuts = Device('Undercuts')

        for turn in range(spiral_params.num_turns):
            Res_Segment = Device('Upper resistive Segment')
            Res_Segment << pg.rectangle(size=(spiral_params.inner_diameter + spiral_params.connector_width+ spiral_params.connector_overlap_x + 2*turn*(spiral_params.turn_spacing_x + spiral_params.connector_width),
                                                spiral_params.res_width),
                                            layer=spiral_params.highdose_layer)

            #Undercut
            # undercut on the left side
            if spiral_params.left_side_undercut:
                Undercut = pg.rectangle(size=(-spiral_params.undercut_margin, spiral_params.undercut_width), layer=spiral_params.undercut_layer)
                Undercut.movey(-(spiral_params.undercut_width/2-spiral_params.res_width/2))
            # undercut on the right side
            else:
                Undercut = pg.rectangle(size=(spiral_params.undercut_margin, spiral_params.undercut_width), layer=spiral_params.undercut_layer)
                Undercut.movex(Res_Segment.xmax)
                Undercut.movey(-(spiral_params.undercut_width/2-spiral_params.res_width/2))

            

            Upper_Segment = Res_Segment + Undercut
            
            Upper_Segment.move(origin=Res_Segment.center, destination=(0, spiral_params.inner_diameter/2 + spiral_params.res_width/2 
                                                                            + turn*(spiral_params.turn_spacing_y +spiral_params.res_width)))
            Upper_Segment.movex(spiral_params.connector_overlap_x/2 - spiral_params.connector_width/2)
            

            Undercuts << Undercut
            ResSegments.append(Res_Segment)

            Res_Segment = Device('Lower resistive Segment')
            Res_Segment << pg.rectangle(size=(spiral_params.inner_diameter + spiral_params.connector_width+spiral_params.connector_overlap_x + (2*turn+1)*(spiral_params.turn_spacing_x + spiral_params.connector_width),
                                                spiral_params.res_width),
                                            layer=spiral_params.highdose_layer)
            
            #Undercut
            # undercut on the left side
            if spiral_params.left_side_undercut:
                Undercut = pg.rectangle(size=(-spiral_params.undercut_margin, spiral_params.undercut_width), layer=spiral_params.undercut_layer)
                Undercut.movey(-(spiral_params.undercut_width/2-spiral_params.res_width/2))
            # undercut on the right side
            else:
                Undercut = pg.rectangle(size=(spiral_params.undercut_margin, spiral_params.undercut_width), layer=spiral_params.undercut_layer)
                Undercut.movex(Res_Segment.xmax)
                Undercut.movey(-(spiral_params.undercut_width/2-spiral_params.res_width/2))

            

            Lower_Segment = Res_Segment + Undercut

            Lower_Segment.move(origin=(Res_Segment.xmin, Res_Segment.ymax), destination=(-spiral_params.inner_diameter/2 - spiral_params.connector_width - spiral_params.turn_spacing_x,
                                                                                                -(spiral_params.inner_diameter/2 + turn*(spiral_params.turn_spacing_y +spiral_params.res_width))))
            Lower_Segment.movex(-spiral_params.connector_overlap_x - turn*(spiral_params.turn_spacing_x + spiral_params.connector_width))
            Lower_Segment.movex(spiral_params.connector_overlap_x - spiral_params.connector_width)
            
            Undercuts << Undercut
            ResSegments.append(Res_Segment)

            Right_Segment = pg.rectangle(size=(spiral_params.connector_width, 
                                                spiral_params.inner_diameter + ((2*turn+1)-1)*(spiral_params.turn_spacing_y+spiral_params.res_width) + 2*spiral_params.res_width + 2*spiral_params.connector_overlap_y),
                                        layer=spiral_params.layer)
            #Undercut
            Right_Segment << pg.rectangle(size=(spiral_params.undercut_width, -spiral_params.undercut_margin), layer=spiral_params.undercut_layer).movex(- spiral_params.undercut_width/2 + spiral_params.connector_width/2)
            Right_Segment << pg.rectangle(size=(spiral_params.undercut_width, spiral_params.undercut_margin), layer=spiral_params.undercut_layer).movey(Right_Segment.ymax).movex(- spiral_params.undercut_width/2 + spiral_params.connector_width/2)

            Right_Segment.move(origin=Right_Segment.center, destination=(spiral_params.inner_diameter/2 + turn*(spiral_params.turn_spacing_x+spiral_params.connector_width) + spiral_params.connector_width/2, 0))


            ConnSegments.append(Right_Segment)

            
            Left_Segment = pg.rectangle(size=(spiral_params.connector_width, spiral_params.inner_diameter + (2*(turn+1)+1)*spiral_params.res_width + (2*(turn+1)-1)*spiral_params.turn_spacing_y+ 2*spiral_params.connector_overlap_y),
                                        layer = spiral_params.layer)
            #Undercut
            Left_Segment << pg.rectangle(size=(spiral_params.undercut_width, -spiral_params.undercut_margin), layer=spiral_params.undercut_layer).movex(- spiral_params.undercut_width/2 + spiral_params.connector_width/2)
            
            if turn < spiral_params.num_turns-1 or spiral_params.undercut_at_end:
                Left_Segment << pg.rectangle(size=(spiral_params.undercut_width, spiral_params.undercut_margin), layer=spiral_params.undercut_layer).movey(Left_Segment.ymax).movex(- spiral_params.undercut_width/2 + spiral_params.connector_width/2)

            Left_Segment.move(origin=(pg.extract(Left_Segment, layers=[spiral_params.layer]).xmin, Left_Segment.ymax - spiral_params.undercut_margin), 
                                destination=(-(spiral_params.inner_diameter/2 + spiral_params.connector_width) - (turn+1)*(spiral_params.turn_spacing_x + spiral_params.connector_width),
                                            spiral_params.inner_diameter/2 + spiral_params.res_width + (turn+1)*(spiral_params.turn_spacing_y + spiral_params.res_width) + spiral_params.connector_overlap_y))
            


            ConnSegments.append(Left_Segment)


        
        

        if spiral_params.connector:
            Connector = Device('Connector')

            Connector << pg.rectangle(size=(spiral_params.connector_width, spiral_params.connector_length), layer=spiral_params.layer)

            Connector.move(origin=(Connector.xmin, Connector.ymin), destination=(pg.extract(ConnSegments[-1], layers=[spiral_params.layer]).xmin, pg.extract(ConnSegments[-1], layers=[spiral_params.layer]).ymax))

            ConnSegments[-1] = pg.boolean(A=pg.extract(ConnSegments[-1], layers=[spiral_params.layer]), B=Connector, operation='A+B', layer=spiral_params.layer)

            Undercut = pg.rectangle(size=(spiral_params.undercut_width, -spiral_params.undercut_margin), layer=spiral_params.undercut_layer)
            Undercut.move(origin=(Undercut.xmin, Undercut.ymax), destination=(ConnSegments[-1].xmin - spiral_params.undercut_width/2 + spiral_params.connector_width/2, ConnSegments[-1].ymin))
            Undercuts << Undercut

        ResSegments = pg.boolean(A=ResSegments, B=ConnSegments, operation='A-B', layer=spiral_params.highdose_layer)

        S << ResSegments
        S << ConnSegments
        S << Undercuts

        return S
    
    def add_ports(self, spiral_params: SquareSpiralParams) -> None:
        self.device.add_port(name='in', midpoint=(self.device.xmin + spiral_params.undercut_margin*spiral_params.left_side_undercut + spiral_params.connector_width/2, self.device.ymax), width=spiral_params.connector_width, orientation=90)
        
      

    def generate_square_spiral(self, spiral_params: SquareSpiralParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Spiral already exists.')
        else:
            self.device = SquareSpiral.create_square_spiral(spiral_params)

            self.add_ports(spiral_params)

            self.spiral_params = spiral_params