from typing import Protocol

from .simulation import *


class ShipController(Protocol):
    def get_controls(self) -> ShipControls:
        ...

class KeyboardController:
    def __init__(self):
        pass

    def get_controls(self) -> ShipControls:
        pass
