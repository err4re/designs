### phidl imports
from phidl import Device, Layer, LayerSet, Path, CrossSection
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
class SquarePortParams:
    w: float = 500 #width of square
    spacing: float = 255 #spacing around square with respect to ground plane

    routing_layer: Layer = default_ls['routing']
    optical_layer: Layer = default_ls['optical']
    ground_avoidance_layer: Layer = default_ls['ground_avoidance']

@dataclass
class FeedlineParams:

    w: float = 10 #width of central line
    s: float = 500 #spacing between line and groundplane
    r: float = 100 #radius for turns

    # feedline_points: list[Tuple] = field(default_factory=lambda: [(0, 0), (0, 500), (1500, 500), (1500, 1300), (3500, 1300), (3500, 500), (5000, 500), (5000, 0)]) #points the feedline follow
    
    feedline_points: list[Tuple] = field(default_factory=lambda: [(0, 0), (0, 1200), (5000, 1200), (5000, 0)]) #points the feedline follow
    device_points: list[Tuple] = field(default_factory=lambda: [(x, 1200) for x in np.linspace(start=100, stop=4900, num=9)]) #points where devices couple to the feedline
    device_ground_points: list[Tuple] = field(default_factory=lambda: [(x, 1200 + 5 + 500) for x in np.linspace(start=100, stop=4900, num=9)]) #points where devices connect to ground
    device_orientations: Optional[list[float]] = None

    # feedline_points: list[Tuple] = field(default_factory=lambda: [(0, 0), (1200, 0), (1200, 1500), (3800, 1500), (3800, 0), (5000, 0)]) #points the feedline follow
    # device_points: list[Tuple] = field(default_factory=lambda: [(1200, 600), (1200, 1000), (1200, 1400), (3800, 1400), (3800, 1000), (3800, 600)]) #points where devices couple to the feedline
    

    mwport_type: str = 'square' #type or shape of microwave ports at the end of the feed line
    port_params: Optional[SquarePortParams] = field(default_factory=SquarePortParams) #parameters for mw port

    ground_avoidance_spacing: int = 100

    routing_layer: Layer = default_ls['routing']
    optical_layer: Layer = default_ls['optical']
    ground_avoidance_layer: Layer = default_ls['ground_avoidance']

    def __post_init__(self):
        valid_port_types = ['square']
        if self.mwport_type not in valid_port_types:
            raise ValueError(f"Unsupported microwave port type: {self.mwport_type}")
    
    
    


class Feedline:
    """
    Class to represent and generate a feedline with its microwave ports.

    Attributes:
    """

    def __init__(self) -> None:
        self.line_params: Optional[FeedlineParams] = None

        self.device: Optional[Device] = None
        self.line: Optional[DeviceReference] = None

        self.left_mwport: Optional[DeviceReference] = None
        self.right_mwport: Optional[DeviceReference] = None

        self.path: Optional[Path] = None


    @staticmethod
    def create_square_mwport(port_params: SquarePortParams) -> Device:
        MW_port = Device('Square microwave port')

        # create pad for wirebonding in routing layer
        Pad = MW_port << pg.rectangle(size=(port_params.w, port_params.w), layer=port_params.routing_layer)

        # create spacing around pad in optical layer
        Offset = pg.offset(elements=Pad, distance=port_params.spacing, layer=port_params.optical_layer)
        MW_port << Offset
        #Spacing = MW_port << pg.boolean(A=Offset, B=Pad, operation='A-B', layer=port_params.optical_layer)

        # add ports
        MW_port.add_port(name='in', midpoint=(-port_params.spacing,port_params.w/2), width=port_params.w, orientation=180)
        MW_port.add_port(name='out', midpoint=(port_params.w, port_params.w/2), width=port_params.w, orientation=0)

        MW_port.add_port(name='in left', midpoint=(port_params.w/2,port_params.w+port_params.spacing), width=port_params.w, orientation=90)
        MW_port.add_port(name='in right', midpoint=(port_params.w/2,-(port_params.spacing)), width=port_params.w, orientation=-90)
        

        return MW_port
    
    
    def get_length(self) -> float:

        return self.path.length()
    
    
    def create_line(self, line_params: FeedlineParams) -> Device:
        
        self.path = pp.smooth(points=line_params.feedline_points, radius=line_params.r, corner_fun=pp.euler, use_eff=False)

        X = CrossSection()
        X.add(width = line_params.w, offset = 0, ports=['in', 'out'], name='routing', layer=line_params.routing_layer)
        X.add(width = 2*line_params.s + line_params.w, offset = 0, name='optical', layer=line_params.optical_layer)
        # X.add(width = line_params.s, offset = (line_params.s + line_params.w)/2, name='optical top', layer=line_params.optical_layer)
        # X.add(width = line_params.s, offset = -(line_params.s + line_params.w)/2, name='optical bot', layer=line_params.optical_layer)
        
        Line = self.path.extrude(X)

        return Line
    
    def create_ground_avoidance(self, line_params: FeedlineParams) -> Device:
        GroundAvoidance = Device('Ground avoidance')

        GroundAvoidance << pg.offset(elements=self.device, distance=line_params.ground_avoidance_spacing, layer=line_params.ground_avoidance_layer)

        return GroundAvoidance
    


    def correct_optical_layer(self) -> None:
        utils.subtract_layers(self.device, self.line_params.optical_layer, self.line_params.routing_layer)
    

    def fix_optical_layer(self, line_params: FeedlineParams) -> None:

        #OpticalPolys = pg.extract(self.device, layers=[line_params.optical_layer])
        OpticalPolys = pg.union(pg.extract(self.device, layers=[line_params.optical_layer]), join_first=True)

        # work around to remove points inside the polygon
        OpticalPolys = pg.offset(elements=OpticalPolys, distance=0.1)
        OpticalPolys = pg.offset(elements=OpticalPolys, distance=-0.1)

        #OpticalPolys = ([poly.simplify(tolerance=1) for poly in OpticalPolys.get_polygonsets()])
        OpticalPolys = ([poly.fillet(line_params.r) for poly in OpticalPolys.get_polygonsets()])
        self.device.remove_layers(layers=[line_params.optical_layer])

        RoutingPolys = pg.extract(self.device, layers=[line_params.routing_layer])

        #[self.device.add_polygon(poly) for poly in OpticalPolys]
        #self.device << OpticalPolys
        self.device << pg.boolean(A=OpticalPolys, B=RoutingPolys, operation='A-B', layer=line_params.optical_layer)

    def add_device_ports(self, line_params: FeedlineParams) -> None:
        if line_params.device_orientations is None:
            [self.device.add_port(name=f'device {i}', midpoint=point, orientation=90) for i,point in enumerate(line_params.device_points)]
            [self.device.add_port(name=f'device {i} ground', midpoint=point, orientation=-90) for i,point in enumerate(line_params.device_ground_points)]
        else:
            [self.device.add_port(name=f'device {i}', midpoint=point, orientation=orientation) for i,(point,orientation) in enumerate(zip(line_params.device_points, line_params.device_orientations))]
            [self.device.add_port(name=f'device {i} ground', midpoint=point, orientation=orientation+180) for i,(point,orientation) in enumerate(zip(line_params.device_ground_points, line_params.device_orientations))]
        
        
                    

    def add_feedline_ports(self) -> None:

        # from left to right
        self.device.add_port(name='in', port=self.left_mwport.ports['in'])
        self.device.add_port(name='out', port=self.right_mwport.ports['in'])

        self.device.add_port(name='in left', port=self.left_mwport.ports['in left']) 
        self.device.add_port(name='in right', port=self.left_mwport.ports['in right']) 

        self.device.add_port(name='out right', port=self.right_mwport.ports['in left']) 
        self.device.add_port(name='out left', port=self.right_mwport.ports['in right']) 
    
    
    def generate_feedline(self, line_params: FeedlineParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Feed line already exists.')
        else:
            self.device = Device('Feed line')

            # create mw ports
            if line_params.mwport_type == 'square':
                self.left_mwport = self.device << Feedline.create_square_mwport(line_params.port_params)
                self.right_mwport = self.device << Feedline.create_square_mwport(line_params.port_params)
            # options for other types of ports

            else:
                raise ValueError(f"Unsupported microwave port type: {line_params.mwport_type}")


            # create line
            self.line = self.device << self.create_line(line_params)

            # connect ports and line
            # position ports at start and end of feed line respectively, line goes from left to right
            self.left_mwport.connect('out', self.line.ports['in'])
            self.right_mwport.connect('out', self.line.ports['out'])

            # fix optical layer
            self.fix_optical_layer(line_params)

            # add feed line ports
            self.add_feedline_ports()

            # add ports for devices
            self.add_device_ports(line_params)   

            # add ground avoidance box
            self.device << self.create_ground_avoidance(line_params)         

            
            self.line_params = line_params