from enum import Enum

class Direction(Enum):
    LEFT = 1
    RIGHT = 2

class State(Enum):
    IDLE = 1
    TURN = 2
    MOVE_FORWARD = 3

class Orientation(Enum):
    NORTH = 1
    EAST = 2
    SOUTH = 3
    WEST = 4

def get_next_orientation(current_orientation):
    next_orientation = Orientation((current_orientation.value + 1) % len(Orientation))
    return next_orientation

def get_prev_orientation(current_orientation):
    if current_orientation == 1:
        return Orientation.WEST

    next_orientation = Orientation((current_orientation.value - 1) % len(Orientation))
    return next_orientation