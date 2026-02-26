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

squid = Squid()

res_params = ResParams(spacing=5, adaptive_spacing=False, num_segments=10, total_spacing=12, segment_length=20, connector_width=2)
squid_params = SquidParams()
capa_params = CapaParams(length_x=20, length_y=8)
writefield_params = WritefieldParams()

squid.generate_squid(squid_params)

squid.device.remove_layers([default_ls['writefield_ebeam'], default_ls['working_area_ebeam']])

set_quickplot_options(show_ports=False, show_subports=False, new_window=True, blocking=True, label_aliases=True)

qp(squid.device)

