### phidl imports
from phidl import Device


from dataclasses import dataclass, field, replace

class Component():

    def __init__(self):

        self.device = Device()