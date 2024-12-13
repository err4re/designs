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
class JJParams:
    bridge_width: float = 0.4
    jj_width: float = 0.2
    jj_length: float = 4

    arm_width: float = 0.22
    total_length: float = 100

    undercut_width: float = 0.5

    extended_undercut: bool = False
    undercut_extension: float = 0.5
    undercut_spacing_v: float = 0.075
    undercut_spacing_h: float = 0.1
    undercut_slit: float = 0.04

    stress_boxes_width: float = 0.1
    stress_boxes_height: float = 1
    stress_boxes_undercut_height: float = 0.5
    stress_boxes_undercut_width: float = 0.2

    undercut_shape: str = 'H_boxes'



    layer: Layer = default_ls['ebeam']
    undercut_layer: Layer = default_ls['jj_undercut']

    def __post_init__(self):
        valid_shapes = ['rectangle', 'H', 'H_rounded', 'H_slit', 'H_boxes', 'H_sym_slits', 'H_asym_slits', 'H_gills','none']
        if self.undercut_shape not in valid_shapes:
            raise ValueError(f"Unsupported undercut shape: {self.undercut_shape}")


### define Josephson Junction class
class JJ:
    """
    Class to represent and generate a Josephson Junction centered around the origin.

    Attributes:
        device (Device): A PHIDL device object representing the junction.
        jj_params (JJParams): Parameters used for junction generation.
        undercut (DeviceReference): A PHIDL device reference object representing the undercut.
        junction (DeviceReference): A PHIDL device reference object representing the arms of the junction.
    """

    def __init__(self) -> None:
        self.jj_params: Optional[JJParams] = None

        self.device: Optional[Device] = None

        self.undercut: Optional[DeviceReference] = None
        self.junction: Optional[DeviceReference] = None

    ### methods to create junction pieces
    @staticmethod
    def create_jj_top(jj_params: JJParams) -> Device:
        JJ_Top = Device('JJ Top')
        JJ_Top << pg.rectangle(size=(jj_params.jj_width, jj_params.jj_length/2 - jj_params.bridge_width/2), layer=jj_params.layer)
        JJ_Top.move((-jj_params.jj_width/2, jj_params.bridge_width/2))

        return JJ_Top
    
    @staticmethod
    def create_jj_bot(jj_params: JJParams) -> Device:
        JJ_Bot = Device('JJ Bot')
        JJ_Bot << pg.rectangle(size=(jj_params.jj_width, jj_params.jj_length/2 - jj_params.bridge_width/2), layer=jj_params.layer)
        JJ_Bot.move((-jj_params.jj_width/2, -jj_params.jj_length/2))

        return JJ_Bot

    @staticmethod
    def create_top_arm(jj_params: JJParams) -> Device:
        Top_Arm = Device('Top Arm')
        Top_Arm << pg.rectangle(size=(jj_params.arm_width, (jj_params.total_length-jj_params.jj_length)/2), layer=jj_params.layer)
        Top_Arm.move((-jj_params.arm_width/2, jj_params.jj_length/2))

        return Top_Arm
    
    @staticmethod
    def create_bot_arm(jj_params: JJParams) -> Device:
        Bot_Arm = Device('Bot Arm')
        Bot_Arm << pg.rectangle(size=(jj_params.arm_width, (jj_params.total_length-jj_params.jj_length)/2), layer=jj_params.layer)
        Bot_Arm.move((-jj_params.arm_width/2, -jj_params.total_length/2))

        return Bot_Arm
    

    ### methods to create undercuts
    @staticmethod
    def create_undercut_rectangle(jj_params: JJParams) -> Device:
        JJ_Undercut = Device('rectangle undercut')

        if jj_params.extended_undercut:
            JJ_Undercut << pg.rectangle(size=(jj_params.jj_width + 2*jj_params.undercut_extension, jj_params.bridge_width - 2*jj_params.undercut_spacing_v), layer=jj_params.undercut_layer).move((-jj_params.jj_width/2 - jj_params.undercut_extension, - jj_params.bridge_width/2 + jj_params.undercut_spacing_v))

        else:
            JJ_Undercut << pg.rectangle(size=(jj_params.jj_width, jj_params.bridge_width - 2*jj_params.undercut_spacing_v), layer=jj_params.undercut_layer).move((-jj_params.jj_width/2, - jj_params.bridge_width/2+ jj_params.undercut_spacing_v))

        return JJ_Undercut

    @staticmethod
    def create_undercut_h(jj_params: JJParams) -> Device:
        JJ_Undercut = Device('H undercut')

        Center = pg.rectangle(size=(jj_params.jj_width + 2*jj_params.undercut_spacing_h, jj_params.bridge_width - 2*jj_params.undercut_spacing_v), layer=jj_params.undercut_layer).move((-jj_params.jj_width/2 -jj_params.undercut_spacing_h, - jj_params.bridge_width/2+ jj_params.undercut_spacing_v))
        Right = pg.rectangle(size=(jj_params.undercut_extension, jj_params.undercut_extension), layer=jj_params.undercut_layer).move((jj_params.undercut_spacing_h + jj_params.jj_width/2, -jj_params.undercut_extension/2))
        Left =  pg.rectangle(size=(jj_params.undercut_extension, jj_params.undercut_extension), layer=jj_params.undercut_layer).move((-jj_params.jj_width/2-jj_params.undercut_extension-jj_params.undercut_spacing_h, -jj_params.undercut_extension/2))

        
        JJ_Undercut = pg.boolean(A=[Center, Right, Left], B=[], operation='or', layer=jj_params.undercut_layer)

        return JJ_Undercut
    
    @staticmethod
    def create_undercut_h_rounded(jj_params: JJParams) -> Device:
        JJ_Undercut = Device('H rounded undercut')

        Center = pg.rectangle(size=(jj_params.jj_width + 2*jj_params.undercut_spacing_h, jj_params.bridge_width - 2*jj_params.undercut_spacing_v), layer=jj_params.undercut_layer).move((-jj_params.jj_width/2 -jj_params.undercut_spacing_h, - jj_params.bridge_width/2+ jj_params.undercut_spacing_v))
        Right = pg.arc(radius=jj_params.undercut_extension/2, width=jj_params.undercut_extension, theta=180, start_angle=270, layer=jj_params.undercut_layer).movex(jj_params.jj_width/2 + jj_params.undercut_spacing_h)
        Left = pg.arc(radius=jj_params.undercut_extension/2, width=jj_params.undercut_extension, theta=180, start_angle=-270, layer=jj_params.undercut_layer).movex(-jj_params.jj_width/2 - jj_params.undercut_spacing_h)

        JJ_Undercut << pg.boolean(A=[Center, Right, Left], B=[], operation='or', layer=jj_params.undercut_layer)

        return JJ_Undercut
    
    @staticmethod
    def create_undercut_h_slit(jj_params: JJParams) -> Device:
        JJ_Undercut =  Device('H slit undercut')

        H = JJ.create_undercut_h(jj_params)

        JJ_Undercut = pg.boolean(A=H, B=pg.rectangle(size=(jj_params.undercut_slit, jj_params.bridge_width)).move((-jj_params.undercut_slit/2, -jj_params.bridge_width/2)), 
                                operation='A-B',
                                layer=jj_params.undercut_layer)
        
        return JJ_Undercut
    
    @staticmethod
    def create_undercut_h_boxes(jj_params: JJParams) -> Device:
        JJ_Undercut = Device('H undercut with boxes')

        H = JJ.create_undercut_h(jj_params)

        Box = pg.rectangle(size=(jj_params.stress_boxes_width, jj_params.stress_boxes_height), layer=jj_params.layer)
        Upper_Undercut = Box << pg.rectangle(size=(jj_params.stress_boxes_undercut_width, jj_params.stress_boxes_undercut_height), layer=jj_params.undercut_layer).movex(-jj_params.stress_boxes_width/2).movey(jj_params.stress_boxes_height)
        Lower_Undercut = Box << pg.rectangle(size=(jj_params.stress_boxes_undercut_width, -jj_params.stress_boxes_undercut_height), layer=jj_params.undercut_layer).movex(-jj_params.stress_boxes_width/2)


        Left_Box = JJ_Undercut << Box
        Left_Box.move((-jj_params.undercut_extension - jj_params.stress_boxes_width/2 - jj_params.jj_width/2 - jj_params.undercut_spacing_h, -jj_params.stress_boxes_height/2))
        Right_Box = JJ_Undercut << Box
        Right_Box.move((jj_params.undercut_extension - jj_params.stress_boxes_width/2 + jj_params.jj_width/2 + jj_params.undercut_spacing_h, -jj_params.stress_boxes_height/2))

        H = pg.boolean(A=H, B=pg.offset([Left_Box, Right_Box], distance=jj_params.undercut_spacing_h), operation='A-B', layer=jj_params.undercut_layer)

        JJ_Undercut << H

        return JJ_Undercut
    
    @staticmethod
    def create_undercut_h_sym_slits(jj_params: JJParams) -> Device:

        JJ_Undercut = Device('H undercut with symmetric slits')

        H = JJ.create_undercut_h(jj_params)

        minimal_width = 0.001

        Slit = pg.rectangle(size=(minimal_width, jj_params.stress_boxes_height), layer=jj_params.layer)
        Slit.rotate(45)
        Slit.move(origin=(Slit.xmax, Slit.ymin), destination=(H.xmax + jj_params.undercut_spacing_h, H.ymax - jj_params.undercut_extension/2 + jj_params.undercut_spacing_v))

        Top_Left_Slit = JJ_Undercut << Slit

        Bot_Left_Slit = JJ_Undercut << Slit
        Bot_Left_Slit.mirror(p1=(1,0), p2=(-1, 0))

        Top_Right_Slit = JJ_Undercut << Slit
        Top_Right_Slit.mirror(p1=(0,1), p2=(0,-1))

        Bot_Right_Slit = JJ_Undercut << Slit
        Bot_Right_Slit.mirror(p1=(1,0), p2=(-1, 0))
        Bot_Right_Slit.mirror(p1=(0,1), p2=(0,-1))

        H = pg.boolean(A=H, B=pg.offset([Top_Left_Slit, Bot_Left_Slit, Top_Right_Slit, Bot_Right_Slit], distance=jj_params.undercut_spacing_h), operation='A-B', layer=jj_params.undercut_layer)

        JJ_Undercut << H

        return JJ_Undercut
    
    @staticmethod
    def create_undercut_h_asym_slits(jj_params: JJParams) -> Device:

        JJ_Undercut = Device('H undercut with asymmetric slits')

        H = JJ.create_undercut_h(jj_params)

        minimal_width = 0.001

        Upper_Slit = pg.rectangle(size=(minimal_width, jj_params.stress_boxes_height), layer=jj_params.layer)
        Upper_Slit.rotate(45)
        Upper_Slit.move(origin=(Upper_Slit.xmax, Upper_Slit.ymin), destination=(H.xmax + jj_params.undercut_spacing_h, H.ymax - jj_params.undercut_extension/2 + jj_params.undercut_spacing_v))

        Top_Left_Slit = JJ_Undercut << Upper_Slit

        Lower_Slit = pg.rectangle(size=(minimal_width, jj_params.stress_boxes_height + np.sqrt(2)*4*jj_params.undercut_spacing_v), layer=jj_params.layer)
        Lower_Slit.rotate(45)
        Lower_Slit.move(origin=(Lower_Slit.xmax, Lower_Slit.ymin), destination=(H.xmax + jj_params.undercut_spacing_h+4*jj_params.undercut_spacing_v, H.ymax - jj_params.undercut_extension/2 + jj_params.undercut_spacing_v-4*jj_params.undercut_spacing_v +jj_params.undercut_spacing_h))


        Bot_Left_Slit = JJ_Undercut << Lower_Slit
        Bot_Left_Slit.mirror(p1=(1,0), p2=(-1, 0))

        Top_Right_Slit = JJ_Undercut << Upper_Slit
        Top_Right_Slit.mirror(p1=(0,1), p2=(0,-1))

        Bot_Right_Slit = JJ_Undercut << Lower_Slit
        Bot_Right_Slit.mirror(p1=(1,0), p2=(-1, 0))
        Bot_Right_Slit.mirror(p1=(0,1), p2=(0,-1))

        H = pg.boolean(A=H, B=pg.offset([Top_Left_Slit, Bot_Left_Slit, Top_Right_Slit, Bot_Right_Slit], distance=jj_params.undercut_spacing_h), operation='A-B', layer=jj_params.undercut_layer)

        JJ_Undercut << H

        return JJ_Undercut


    def _create_undercut(self, jj_params: JJParams) -> DeviceReference:
        """
        Creates the undercut based on the given parameters and adds it to self.device.

        Args:
            jj_params (JJParams): The parameters for the junction.

        Returns:
            DeviceReference: The created undercut device reference.
        """
        undercut_methods = {
            "rectangle": JJ.create_undercut_rectangle,
            "H": JJ.create_undercut_h,
            "H_rounded": JJ.create_undercut_h_rounded,
            "H_slit": JJ.create_undercut_h_slit,
            "H_boxes": JJ.create_undercut_h_boxes,
            "H_sym_slits": JJ.create_undercut_h_sym_slits,
            "H_asym_slits": JJ.create_undercut_h_asym_slits,
            "none": None
        }

        if jj_params.undercut_shape not in undercut_methods:
            raise ValueError(f"Unsupported undercut shape: {jj_params.undercut_shape}")

        method = undercut_methods[jj_params.undercut_shape]
        if method is None:
            return None
        return self.device << method(jj_params)



    ### add ports to component for later use
    def add_ports(self, jj_params: JJParams) -> None:

        self.device.add_port(name='top', midpoint=((self.device.xmax + self.device.xmin)/2, self.device.ymax), width=jj_params.arm_width, orientation=90)
        self.device.add_port(name='bot', midpoint=((self.device.xmax + self.device.xmin)/2, self.device.ymin), width=jj_params.arm_width, orientation=-90)




    ### method to generate full geometry
    def generate_jj(self, jj_params: JJParams, overwrite: bool = False) -> None:
        if (self.device is not None) and not overwrite:
            print('JJ already exists.')
        else:
            self.device = Device('Josephson Junction')

            # Generate pieces of the junction
            JJ_Top = JJ.create_jj_top(jj_params)
            JJ_Bot = JJ.create_jj_bot(jj_params)
            Top_Arm = JJ.create_top_arm(jj_params)
            Bot_Arm = JJ.create_bot_arm(jj_params)

            # Unite all shapes into one polygon
            Junction = Device('Junction')
            Junction << pg.boolean(A=[JJ_Top, JJ_Bot, Top_Arm, Bot_Arm], B=[], operation='or', layer=jj_params.layer)
            self.junction = self.device << Junction

            # Generate the undercut shape based on the parameter
            self.undercut = self._create_undercut(jj_params)

            # Create ports for future use
            self.add_ports(jj_params)
                
            self.jj_params = jj_params


