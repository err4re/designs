from components.spiral import Spiral
from components.squid import Squid
from components.squid_resistor import SquidResistor
from components.junction_resistor import JJResistor
from components.junction_squid_resistor import JJSquidResistor

from phidl import Device

from typing import Union, Optional

from dataclasses import dataclass

@dataclass
class QRCSJDevice:

    assembled_device : Optional[Device] = None
    spiral: Optional[Spiral] = None
    qubit: Optional[Union[Squid, SquidResistor, JJResistor, JJSquidResistor]] = None

    position: Optional[tuple] = None
    number: Optional[int] = None
    id: Optional[str] = None # e.g. Right 1 

    def get_info(self) -> dict:
        info = {'id' : self.id,
                'number': self.number,
                'spiral frequency': self.spiral.get_resonance_frequency(),
                'qubit': type(self.qubit),
                'position': self.position}


    def print_info(self):
        info = self.get_info()
        for key,value in info.items():
            print(f'{key}: {value}')
            
    

