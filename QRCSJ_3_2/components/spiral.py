### phidl imports
from phidl import Device, Layer, LayerSet, Path
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

### scipy
import scipy.constants

### component imports
from components import utils
from components.default_layerset import default_ls

def spiral_points(D, p, N, n):
    """
    Generates a set of Cartesian coordinates for a Archimedean spiral based on given parameters.

    Parameters:
        D (float): Diameter of the spiral at its origin (starting point).
        p (float): Pitch of the spiral, which determines the distance between each turn.
        N (float): Total number of turns in the spiral. Can be a non-integer to specify a partial final turn.
        n (int): Number of points to calculate along the spiral. Higher values result in a smoother spiral.

    Returns:
        list of tuple: A list of (x, y) tuples representing the Cartesian coordinates of the spiral.

    """

    # Calculate the range of theta for the specified number of turns
    theta_max = N * 2 * np.pi
    theta = np.linspace(0, theta_max, n)

    b = p/(2*np.pi)

    # Calculate the corresponding values of r(theta)
    r = D/2 + b * theta

    # Convert polar coordinates to Cartesian coordinates
    x = r * np.cos(theta) 
    y = r * np.sin(theta)

    spiral_points = [[xi, yi] for xi, yi in zip(x,y)]

    #fix up points... rotation, shift and cut off end

    #rotation
    rot_angle = np.arctan((spiral_points[1][0] - spiral_points[0][0])/ (spiral_points[1][1] - spiral_points[0][1]))
    R = np.array([[np.cos(rot_angle), -np.sin(rot_angle)],
                  [np.sin(rot_angle),  np.cos(rot_angle)]])
    spiral_points = [(R @ point) for point in spiral_points]

    #shift to center
    spiral_points = [(point[0] + (spiral_points[0][0] - D/2), point[1] - spiral_points[0][1]) for point in spiral_points]

    #cut off end
    if N % 1 == 0:

        i = len(spiral_points) - 1

        while spiral_points[i][1] > 0:
            i = i-1
            
    else:
        angle_final = (N % 1)*360

        i = len(spiral_points) - 1

        current_angle = (np.degrees(np.arctan2(spiral_points[i][1], spiral_points[i][0])) + 360) % 360
        while current_angle > angle_final:
            i = i-1
            current_angle = (np.degrees(np.arctan2(spiral_points[i][1], spiral_points[i][0])) + 360) % 360

    spiral_points = spiral_points[:(i+2)] 


    return spiral_points

### define parameters
@dataclass
class SpiralParams:

    d_inner: float = 10  # Inner diameter
    p: float = 6  # pitch between turns
    w: float = 3  # width of central line
    N: float = 17  # Number of turns

    n: int = 1500   #number of points

    spiral_points: list = field(default_factory=list, init=False)  # List of points, initialized post creation

    connector_length: float = 100
    connector_shift: float = 0

    layer: Layer = default_ls['routing']

    def __post_init__(self):
        self.spiral_points = spiral_points(self.d_inner, self.p, self.N, self.n)


### define spiral class
class Spiral:
    """
    Class to represent and generate a spiral centered around the origin.

    Attributes:
        device (Device): A PHIDL device object representing the spiral.
        spiral_params (SpiralParams): Parameters used for spiral generation.
        
    """

    def __init__(self) -> None:
        self.spiral_params: SpiralParams = None
        self.device: Device = None
        self.connector: DeviceReference = None

        self.path: Optional[Path] = None

    @staticmethod
    def create_spiral(spiral_params: SpiralParams) -> Tuple[Device, Path]:
        P = Path(spiral_params.spiral_points)

        # fix orientation, spiral tail faces right side
        P.rotate(angle=-(spiral_params.N % 1)*360)
        S = P.extrude(width=spiral_params.w, layer=spiral_params.layer)

        return S, P
    
    
 
    def create_connector(self, spiral_params: SpiralParams) -> Device:

        spiral_points = self.path.points
        closest_spiral_point = utils.find_closest_point_in_x_with_y_tolerance((self.device.xmin, spiral_params.connector_shift), spiral_points, y_tolerance=spiral_params.w)
        extra_length = np.abs(closest_spiral_point[0] - self.device.xmin)

        Connector = Device('Connector')
        Compass = Connector << pg.compass(size=(spiral_params.connector_length + extra_length, spiral_params.w), layer= spiral_params.layer)
        for port in Compass.ports.values():
            Connector.add_port(port)

        
        
        Connector.move(origin=Connector.ports['E'], destination=(self.device.xmin + extra_length , spiral_params.connector_shift))
        
        return Connector

    def get_length(self) -> float:

        return self.path.length()
    
    
    def get_resonance_frequency(self, offset: float = 1.2e9) -> float:
        if self.spiral_params is None:
            print('Generate spiral or set spiral parameters first')
        else:
            res_freq = Spiral.fg(p=self.spiral_params.p, N=self.spiral_params.N, d_in=self.spiral_params.d_inner) - offset

            return res_freq
    

    @staticmethod
    def fg(p: float, N: float, d_in: float) -> float:
        
        c0 = scipy.constants.speed_of_light
        xi = 0.81
        epsilon_eff = 6

        return xi * c0/np.sqrt(epsilon_eff) * (2e-6*p)/(np.pi * ((d_in + 2*N*p)*1e-6)**2)

    @staticmethod
    def Lg(p: float, N: float, d_in: float) -> float:
        mu0 = scipy.constants.mu_0
        
        din = d_in
        dout = din + 2*p*N
        dav = (din+dout)/2

        rho = (dout - din)/(din + dout)

        return (mu0 *N**2 *dav*1e-6)/2 * (np.log(2.5/rho) + 0.2* rho**2) 
    

    def add_ports(self, spiral_params: SpiralParams) -> None:
        self.device.add_port(name='in', midpoint=(self.device.xmax, 0), orientation=0)
        self.device.add_port(name='out', midpoint=(self.device.xmin, 0), orientation=180)

        if spiral_params.connector_length > 0:
            self.device.add_port(name='capa left', port=self.connector.ports['N'])
            self.device.add_port(name='capa right', port=self.connector.ports['S'])

    def generate_spiral(self, spiral_params: SpiralParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Spiral already exists.')
        else:
            self.device = Device('Spiral')

            spiral, path = Spiral.create_spiral(spiral_params)
            self.device << spiral
            self.path = path

            if spiral_params.connector_length > 0:
                connector = self.create_connector(spiral_params)
                self.connector = self.device << connector

                utils.unify_layer(self.device, spiral_params.layer)

            self.add_ports(spiral_params)

            self.spiral_params = spiral_params




    