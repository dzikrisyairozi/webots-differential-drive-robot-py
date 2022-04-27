from enum import Enum

class Direction(Enum):
    LEFT = 1
    RIGHT = 2

class State(Enum):
    IDLE = 1
    TURN = 2