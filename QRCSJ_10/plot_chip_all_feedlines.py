from phidl import Device, Layer, LayerSet
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

from typing import Tuple, Optional

import itertools

import importlib

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
from components import utils
importlib.reload(utils)

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


SAMPLE_NAME ='QRCSJ 7'
SAMPLE = 'QRCSJ'
SAMPLE_NUMBER = '7'

Chip = Device('Chip')


frame = Frame()
frame.generate_frame(FrameParams(sample_name=SAMPLE_NAME))

Chip << frame.device

set_quickplot_options(show_ports=True, show_subports=False, new_window=True, blocking=True, label_aliases=False)
with open('feedline_top.pkl', 'rb') as f:
    feedline_top = pickle.load(f)

Feedline_Top = Chip << feedline_top.device

Feedline_Top.connect('out', frame.device.ports['pcb 6'])

with open('feedline_bot.pkl', 'rb') as f:
    feedline_bot = pickle.load(f)

Feedline_Bot = Chip << feedline_bot.device

Feedline_Bot.connect('in', frame.device.ports['pcb 1'])

with open('feedline_right.pkl', 'rb') as f:
    feedline_right = pickle.load(f)

Feedline_Right = Chip << feedline_right.device

Feedline_Right.connect('in right', frame.device.ports['pcb 3'])

with open('feedline_left.pkl', 'rb') as f:
    feedline_left: Feedline = pickle.load(f)

Feedline_Left = Chip << feedline_left.device

Feedline_Left.connect('in right', frame.device.ports['pcb 7'])


qp(Chip)