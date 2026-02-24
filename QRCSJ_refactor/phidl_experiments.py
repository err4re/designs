from phidl import Device, quickplot
import phidl.geometry as pg
from phidl import set_quickplot_options

import numpy as np
import matplotlib.pyplot as plt

# --- make a simple device ---
D = Device("demo")

rect = pg.rectangle(size=(20, 10), layer=(1, 0))
D.add_ref(rect)

# optional: add ports so you can see them
D.add_port(
    name="p1",
    midpoint=(0, 5),
    width=10,
    orientation=180,
)

# --- configure quickplot behavior ---
set_quickplot_options(
    new_window=True,     # open a new window
    blocking=False,      # don't halt script execution
    show_ports=True,     # draw ports
    show_subports=False, # don't show ports of references
)

# --- show it ---
quickplot(D)
plt.show()