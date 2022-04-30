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
    NORTH_EAST = 2
    EAST = 3
    SOUTH_EAST = 4
    SOUTH = 5
    SOUTH_WEST = 6
    WEST = 7
    NORTH_WEST = 8

def get_next_orientation(current_orientation):
    next_orientation = Orientation((current_orientation.value + 1) % len(Orientation))
    return next_orientation

def get_prev_orientation(current_orientation):
    if current_orientation == 1:
        return Orientation(len(Orientation))

    next_orientation = Orientation((current_orientation.value - 1) % len(Orientation))
    return next_orientation
