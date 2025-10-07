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
class FrameParams:
    sample_name: str = 'Sample name'
    sample_name_size: int = 250

    chip_size: int = 10000
    
    dicing_reference_outer_size: int = 10000
    dicing_reference_width: int = 200

    pcb_pad_width: int = 900
    pcb_pad_height: int = 500
    pcb_pad_number: int = 8
    pcb_pad_corner_spacing: int = 2050
    pcb_mw_spacing: int = 400

    pcb_pad_positions: list[tuple[int, int]] = field(default_factory=list)

    #Corner_Mark: Device = None

    ground_avoidance_spacing: int = 60

    optical_layer: Layer = default_ls['optical']
    pcb_layer: Layer = default_ls['pcb']
    ground_avoidance_layer: Layer = default_ls['ground_avoidance']

    def __post_init__(self):
        
        if self.dicing_reference_outer_size > self.chip_size:
            raise ValueError(f"Dicing reference does not fit inside chip, increase chip size.")
        
        self.pcb_pad_positions = [(-self.pcb_pad_height, self.pcb_pad_corner_spacing + self.pcb_pad_width), (self.pcb_pad_corner_spacing, -self.pcb_pad_height), 
                                (self.pcb_pad_corner_spacing + self.chip_size//2, -self.pcb_pad_height), (self.chip_size + self.pcb_pad_height, self.pcb_pad_corner_spacing),
                                (self.chip_size + self.pcb_pad_height, self.pcb_pad_corner_spacing + self.chip_size//2), (self.pcb_pad_corner_spacing + self.chip_size//2 + self.pcb_pad_width, self.chip_size + self.pcb_pad_height), 
                                (self.pcb_pad_corner_spacing + self.pcb_pad_width, self.chip_size + self.pcb_pad_height), (-self.pcb_pad_height, self.pcb_pad_corner_spacing + self.chip_size//2 + self.pcb_pad_width)]


### define Frame class
class Frame:
    """
    Class to represent and generate a frame for a single chip.

    Attributes:
        device (Device): A PHIDL device object representing the frame.
        frame_params (FrameParams): Parameters used for the frame generation.
        dicing_reference (DeviceReference): A PHIDL device reference object representing the dicing reference frame.
        pcb_pads (list[DeviceReference]): A list of PHIDL device reference objects representing the PCB pads with ports to align other structures.
        corner_marks (list[DeviceReference]): A list of PHIDL device reference objects representing the corner marks.
        sample_name (DeviceReference): A PHIDL device reference object representing the label designating the chip.
    """

    def __init__(self) -> None:
        self.frame_params: Optional[FrameParams] = None

        self.device: Optional[Device] = None

        self.dicing_reference: Optional[DeviceReference] = None
        self.pcb_pads: Optional[list[DeviceReference]] = None
        self.corner_marks: Optional[list[DeviceReference]] = None
        self.sample_name: Optional[DeviceReference] = None

    ### create dicing reference
    @staticmethod
    def create_dicing_reference(frame_params: FrameParams) -> Device:
        Dicing = Device('Dicing reference')

        Outer = pg.rectangle(size=(frame_params.dicing_reference_outer_size,frame_params.dicing_reference_outer_size), layer=frame_params.optical_layer)
        Inner = pg.rectangle(size=(frame_params.dicing_reference_outer_size - 2*frame_params.dicing_reference_width,frame_params.dicing_reference_outer_size - 2*frame_params.dicing_reference_width), layer=frame_params.optical_layer)

        Outer.move(origin=Outer.center, destination=(0, 0))
        Inner.move(origin=Inner.center, destination=(0, 0))

        Reference_Frame = Dicing << pg.boolean(A=Outer, B=Inner, operation='not', precision=1e-6, layer=frame_params.optical_layer)
        
        Outer = pg.rectangle(size=(frame_params.chip_size + 2*frame_params.pcb_pad_height,frame_params.chip_size + 2*frame_params.pcb_pad_height), layer=frame_params.ground_avoidance_layer)
        Inner = pg.rectangle(size=(frame_params.dicing_reference_outer_size - 2*frame_params.dicing_reference_width, frame_params.dicing_reference_outer_size - 2*frame_params.dicing_reference_width), layer=frame_params.ground_avoidance_layer)
        
        Outer.move(origin=Outer.center, destination=(0, 0))
        Inner.move(origin=Inner.center, destination=(0, 0))

        Ground_Avoidance = Dicing << pg.boolean(A=Outer, B=Inner, operation='not', precision=1e-6, layer=frame_params.ground_avoidance_layer)

        # # create frame for alignment when dicing chip
        # Outer = pg.rectangle(size=(frame_params.dicing_reference_outer_size,frame_params.dicing_reference_outer_size), layer=frame_params.optical_layer)
        # Outer.move(destination=(frame_params.dicing_reference_width,frame_params.dicing_reference_width))
        # Inner = pg.rectangle(size=(frame_params.dicing_reference_outer_size - 2*frame_params.dicing_reference_width,frame_params.dicing_reference_outer_size - 2*frame_params.dicing_reference_width), layer=frame_params.optical_layer)
        # Inner.move(destination=(2*frame_params.dicing_reference_width, 2*frame_params.dicing_reference_width))

        # Dicing << pg.boolean(A=Outer, B=Inner, operation='not', precision=1e-6, layer=frame_params.optical_layer)
        
        # # create ground avoidance
        # Outer = pg.rectangle(size=(frame_params.chip_size + 2*frame_params.pcb_pad_height,frame_params.chip_size + 2*frame_params.pcb_pad_height), layer=frame_params.ground_avoidance_layer)
        # Outer.move(destination=(-frame_params.pcb_pad_height,-frame_params.pcb_pad_height))
        # Inner = pg.rectangle(size=(frame_params.dicing_reference_outer_size - 2*frame_params.dicing_reference_width, frame_params.dicing_reference_outer_size - 2*frame_params.dicing_reference_width), layer=frame_params.ground_avoidance_layer)
        # Inner.move(destination=(2*frame_params.dicing_reference_width, 2*frame_params.dicing_reference_width))
        
        # Ground_Avoidance = pg.boolean(A=Outer, B=Inner, operation='not', precision=1e-6, layer=frame_params.ground_avoidance_layer)
        
        # Dicing << Ground_Avoidance

        return Dicing
    
    ### create PCB pad references
    @staticmethod
    def create_pcb_pad(frame_params: FrameParams) -> Device:
        PCB_Pad = Device('PCB pad')

        PCB_Pad << pg.rectangle(size=(frame_params.pcb_pad_width,frame_params.pcb_pad_height), layer=frame_params.pcb_layer)
        PCB_Pad.add_port(name='out', midpoint=(frame_params.pcb_pad_width/2, frame_params.pcb_pad_height + frame_params.pcb_mw_spacing), width=frame_params.pcb_pad_width, orientation=90)
        
        return PCB_Pad


    @staticmethod
    def create_pcb_pads(frame_params: FrameParams) -> list[Device]:
        
        PCB_Pads = [Frame.create_pcb_pad(frame_params) for i in range(frame_params.pcb_pad_number)]

        PCB_Pads[0].rotate(angle=-90).move(frame_params.pcb_pad_positions[0])
        PCB_Pads[1].move(frame_params.pcb_pad_positions[1])
        PCB_Pads[2].move(frame_params.pcb_pad_positions[2])
        PCB_Pads[3].rotate(angle=90).move(frame_params.pcb_pad_positions[3])
        PCB_Pads[4].rotate(angle=90).move(frame_params.pcb_pad_positions[4])
        PCB_Pads[5].rotate(180).move(frame_params.pcb_pad_positions[5])
        PCB_Pads[6].rotate(180).move(frame_params.pcb_pad_positions[6])
        PCB_Pads[7].rotate(angle=-90).move(frame_params.pcb_pad_positions[7])

        return PCB_Pads
    
    ### create sample name
    @staticmethod
    def create_sample_name(frame_params: FrameParams) -> Device:

        Sample_Name = Device('Sample name')

        Sample_Name << pg.text(text=frame_params.sample_name, size=frame_params.sample_name_size, justify='center', layer=frame_params.optical_layer)
        Sample_Name.move((frame_params.chip_size/2, 2*frame_params.sample_name_size))

        Ground_Avoidance = pg.offset(Sample_Name, distance=frame_params.ground_avoidance_spacing, layer=frame_params.ground_avoidance_layer)
        Sample_Name << Ground_Avoidance

        return Sample_Name
    
    ### create corner marks
    @staticmethod
    def create_corner_mark(frame_params: FrameParams) -> Device:

        Corner_Mark = Device('Corner mark')
            
        [Corner_Mark << pg.rectangle(size=(30,30), layer=frame_params.optical_layer).move((i*40,i*40)) for i in range(15)]
        [Corner_Mark << pg.rectangle(size=(30,30), layer=frame_params.optical_layer).move((800 + i*40, 800 + i*40)) for i in range(4)]
        [Corner_Mark << pg.rectangle(size=(30,30), layer=frame_params.optical_layer).move((800 + i*40, 800 + i*40)) for i in range(4)]
        [Corner_Mark << pg.rectangle(size=(30,30), layer=frame_params.optical_layer).move((800 + i*40, 14*40 - i*40)) for i in range(4)]
        [Corner_Mark << pg.rectangle(size=(30,30), layer=frame_params.optical_layer).move((14*40 - i*40, 800 + i*40)) for i in range(4)]

        Corner_Mark << pg.boolean(A=pg.rectangle(size=(240,240), layer=frame_params.optical_layer), 
                                    B=pg.rectangle(size=(210,210), layer=frame_params.optical_layer).move((15,15)), 
                                    operation='not', precision=1e-6, layer=frame_params.optical_layer).move((14*40 +15, 14*40 +15))

        #large and small square for alignment
        Positive_Alignment_Mark = pg.rectangle(size=(30,30), layer=frame_params.optical_layer).move((14*40 +15 +120, 14*40 +15 +120))
        Positive_Alignment_Mark << pg.rectangle(size=(10,10), layer=frame_params.optical_layer).move((14*40 +15 +110, 14*40 +15 +110))

        Negative_Alignment_Mark = pg.boolean(A=pg.rectangle(size=(60,60)).move(origin=(30,30),destination=Positive_Alignment_Mark.center),
                                             B=Positive_Alignment_Mark,
                                             operation='A-B',
                                             layer=frame_params.optical_layer)
        
        Cross = Device('Alignment Cross')

        # Define the dimensions of the alignment cross
        cross_length = 12  # Length of the cross arms in microns
        cross_width = 1.5  # Width of the cross arms in microns

        # Create the vertical part of the cross
        vertical = Cross.add_ref(pg.rectangle(size=(cross_width, cross_length), layer=frame_params.optical_layer))
        vertical.center = (0, 0)  # Center the vertical part at the origin

        # Create the horizontal part of the cross
        horizontal = Cross.add_ref(pg.rectangle(size=(cross_length, cross_width), layer=frame_params.optical_layer))
        horizontal.center = (0, 0)  # Center the horizontal part at the origin

        utils.unify_layer(Cross, frame_params.optical_layer)

        Negative_Alignment_Mark << Cross.move((14*40 +15 +120 +15, 14*40 +15 +120 +15))
        
        Corner_Mark << Negative_Alignment_Mark

        #cross for alignment    
        #Corner_Mark << pg.boolean(A=pg.cross(length=120, width=10, layer=frame_params.optical_layer).rotate(45),
        #                             B=pg.rectangle(size=(80,80), layer=frame_params.optical_layer).move((-100,-100)), 
        #                             operation='not', precision=1e-6, layer=frame_params.optical_layer).move((14*40 +15 +120, 14*40 +15 +120))

        Corner_Mark << pg.L(width=10, size=(50,50), layer=frame_params.optical_layer).rotate(180).move((14*40 +15 +120 +80, 14*40 +15 +120 +80))
        Corner_Mark << pg.L(width=10, size=(50,50), layer=frame_params.optical_layer).rotate(180).move((14*40 +15 +120 -40, 14*40 +15 +120 -40))

        #ground avoidance for marks
        Ground_Avoidance = pg.offset(Corner_Mark, distance=frame_params.ground_avoidance_spacing, layer=frame_params.ground_avoidance_layer)
        Corner_Mark << Ground_Avoidance 

        return Corner_Mark
    
    @staticmethod
    def create_corner_marks(frame_params: FrameParams) -> list[Device]:

        #create marks, going from bottom left (bl) to top left (tl)
        Mark_bl = Frame.create_corner_mark(frame_params)
        Mark_br = Frame.create_corner_mark(frame_params)
        Mark_tr = Frame.create_corner_mark(frame_params)
        Mark_tl = Frame.create_corner_mark(frame_params)

        Marks = Mark_bl + Mark_br + Mark_tl + Mark_tr
        #Marks.move((2*frame_params.dicing_reference_width, 2*frame_params.dicing_reference_width))
        Marks.move(origin=(pg.extract(Mark_bl, layers=[frame_params.optical_layer]).xmin, pg.extract(Mark_bl, layers=[frame_params.optical_layer]).ymin), destination=(-frame_params.dicing_reference_outer_size/2+frame_params.dicing_reference_width, -frame_params.dicing_reference_outer_size/2+frame_params.dicing_reference_width))

        #position marks
        # Mark_br.mirror(p1=(frame_params.chip_size/2,-1), p2=(frame_params.chip_size/2,1))
        # Mark_tr.mirror(p1=(0, frame_params.chip_size), p2=(frame_params.chip_size, 0))
        # Mark_tl.rotate(90).movex(frame_params.chip_size).mirror(p1=(0, 0), p2=( frame_params.chip_size,  frame_params.chip_size))

        Mark_br.mirror(p1=(0,-1), p2=(0,1))
        Mark_tr.mirror(p1=(1, -1), p2=(-1, 1))
        Mark_tl.mirror(p1=(-1, 0), p2=(1, 0))



        return [Mark_bl, Mark_br, Mark_tr, Mark_tl]
    
    def add_pcb_ports(self) -> None:

        [self.device.add_port(name=f'pcb {i}', port=pcb.ports['out']) for i, pcb in enumerate(self.pcb_pads)]


    def generate_frame(self, frame_params: FrameParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('Frame already exists.')
        else:
            self.device = Device('Frame')

            self.pcb_pads = self.device << Frame.create_pcb_pads(frame_params)

           

            self.sample_name = self.device << Frame.create_sample_name(frame_params)


            # add ports for pcbs
            self.add_pcb_ports()


            #move to shift (0,0) to center, convention for lithography
            self.device.move(destination = (-frame_params.chip_size/2, -frame_params.chip_size/2))

            self.dicing_reference = self.device << Frame.create_dicing_reference(frame_params)

            self.corner_marks = self.device << Frame.create_corner_marks(frame_params)

            self.frame_params = frame_params