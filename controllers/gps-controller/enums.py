from enum import Enum

class Direction(Enum):
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3

class State(Enum):
    IDLE = 1
    TURN = 2
    MOVE_FORWARD = 3

class Compass(Enum):
    NORTH = 1
    EAST = 2
    SOUTH = 3
    WEST = 4