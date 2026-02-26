from phidl import Device, Layer, LayerSet, Port
from phidl.device_layout import DeviceReference
from phidl import quickplot as qp
from phidl import set_quickplot_options

import phidl.geometry as pg
import phidl.utilities as pu
import phidl.routing as pr
import phidl.path as pp

import numpy as np
import pickle

from dataclasses import dataclass, field, replace

import copy

from typing import Tuple, Optional, Union

import itertools
import importlib

from components import utils
importlib.reload(utils)

from components import qrcsj_device
importlib.reload(qrcsj_device)

from components import default_layerset
importlib.reload(default_layerset)
from components import frame
importlib.reload(frame)
from components import feedline
importlib.reload(feedline)
from components import spiral
importlib.reload(spiral)
from components import junction
importlib.reload(junction)
from components import resistor
importlib.reload(resistor)
from components import junction_resistor
importlib.reload(junction_resistor)
from components import ground_capacitor
importlib.reload(ground_capacitor)
from components import squid_resistor
importlib.reload(squid_resistor)
from components import junction_squid_resistor
importlib.reload(junction_squid_resistor)
from components import squid
importlib.reload(squid)

from components.qrcsj_device import QRCSJDevice
from components.default_layerset import default_ls
from components.frame import Frame, FrameParams
from components.feedline import Feedline, FeedlineParams, SquarePortParams
from components.spiral import Spiral, SpiralParams
from components.junction import JJ, JJParams
from components.resistor import Resistor, ResParams
from components.junction_resistor import JJResistor, CapaParams
from components.ground_capacitor import GroundCapa, GroundCapaParams
from components.squid_resistor import SquidResistor, SquidParams
from components.junction_squid_resistor import JJSquidResistor
from components.squid import Squid
from components.utils import WritefieldParams

import sys
sys.path.insert(0, '../') # add the parent directory of components to sys.path

import importlib

from components import spiral
importlib.reload(spiral)
from components.spiral import Spiral, SpiralParams

from phidl import set_quickplot_options

from phidl import quickplot as qp
spiral = Spiral()
spiral.generate_spiral(spiral_params=SpiralParams(N=16.28, connector_shift=40))

# spiral.device.rotate(90)

import sys
sys.path.insert(0, '../') # add the parent directory of components to sys.path


import importlib

from phidl import set_quickplot_options

from components import resistor
importlib.reload(resistor)
from components.resistor import Resistor, ResParams

from components import squid
importlib.reload(squid)
from components.squid import Squid, SquidParams

from components import squid_resistor
importlib.reload(squid_resistor)
from components.squid_resistor import SquidResistor, CapaParams, WritefieldParams

from phidl import quickplot as qp

from components.default_layerset import default_ls

squid_resistor = SquidResistor()

res_params = ResParams(spacing=5, adaptive_spacing=False, num_segments=10, total_spacing=12, segment_length=20, connector_width=2)
squid_params = SquidParams()
capa_params = CapaParams(length_x=20, length_y=8)
writefield_params = WritefieldParams()

squid_resistor.generate_squid_resistor(res_params, squid_params, capa_params, writefield_params)

squid_resistor.device.remove_layers([default_ls['writefield_ebeam'], default_ls['working_area_ebeam']])


def connect_to_resonator(element: Union[JJResistor, SquidResistor, JJSquidResistor, Squid, JJ],
                         element_port_name: str,
                         spiral: Spiral,
                         resonator_port_name: str,
                         capa_distance: float = 2) -> None:

    resonator = spiral.device
    element_port: Port = element.device.ports[element_port_name]

    resonator_normal = resonator.ports[resonator_port_name].normal[1] - resonator.ports[resonator_port_name].normal[0]
    element_normal = element_port.normal[1] - element_port.normal[0]

    connector_parallel = [-resonator_normal[1], resonator_normal[0]]

    angle = np.round((360/(2*np.pi)) * np.arccos(np.clip(np.dot(resonator_normal,-element_normal), -1, 1)))%360

    if angle != 0:
        element.device.rotate(-angle)
        element.device.mirror()

    element.device.move(element_port, resonator.ports[resonator_port_name])
    element.device.move((capa_distance) * resonator_normal)
    element.device.movex(origin=element_port.x, destination=resonator.ports['out'].x)
    element.device.movex(element_port.width/2)

squid_resistor.device.mirror()
connect_to_resonator(squid_resistor, 'capa bot', spiral, 'capa right')

Spiral_quantum_circuit = Device()
Spiral_quantum_circuit << [spiral.device, squid_resistor.device]
Spiral_quantum_circuit.rotate(90)

set_quickplot_options(show_ports=False, show_subports=False, new_window=True, blocking=True, label_aliases=True)
qp(Spiral_quantum_circuit)

# print(spiral.spiral_params.spiral_points[-1])
# print(spiral.device.bbox)

# print(f'resonance frequency: {spiral.get_resonance_frequency()}')

