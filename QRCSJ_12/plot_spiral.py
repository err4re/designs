import sys
sys.path.insert(0, '../') # add the parent directory of components to sys.path

import importlib

from components import spiral
importlib.reload(spiral)
from components.spiral import Spiral, SpiralParams

from phidl import set_quickplot_options

from phidl import quickplot as qp

spiral = Spiral()
spiral.generate_spiral(spiral_params=SpiralParams(N=3, connector_length=0, d_inner=20, p=12, w=6))
spiral.device.rotate(90)

set_quickplot_options(show_ports=True, show_subports=True, new_window=True, blocking=True, label_aliases=True)
qp(spiral.device)

# print(spiral.spiral_params.spiral_points[-1])
# print(spiral.device.bbox)

# print(f'resonance frequency: {spiral.get_resonance_frequency()}')

