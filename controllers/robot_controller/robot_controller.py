"""differential-drive-robot-controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from a_star import get_route
from controller import Robot as WebotsRobot, GPS, Keyboard, InertialUnit
from enums import *
from math import pi

if __name__ == "__main__":

    # create the Robot instance.
    robot = WebotsRobot()

    # get the time step of the current world.
    TIMESTEP = 32
    max_speed = 6.28 #angular velocity

    # You should insert a getDevice-like function in order to get the
    gps = GPS('gps')
    gps.enable(TIMESTEP)

    keyboard = Keyboard()
    keyboard.enable(TIMESTEP)

    imu = InertialUnit('inertial unit')
    imu.enable(TIMESTEP)

    #Motor Instances
    left_motor = robot.getDevice('motor_1')
    right_motor = robot.getDevice('motor_2')

    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    x, y, z = gps.getValues()
    prev_position = (x, y)

    orientation = Orientation.NORTH
    A_COMPENSATION = 0.5

    TILE_SIZE = 0.25

    current_state = State.IDLE
    prev_state = None
    initial = False

    route = get_route((0, 0), (19, 19))
    current_x, current_y = route.pop(0)
    state_queue = []

    def next_state(next_state):
        global current_state, prev_state, initial
        prev_state = current_state
        current_state = next_state
        initial = True
    
    def update_orientation(direction):
        global orientation
        orientation = get_next_orientation(orientation) if direction == Direction.RIGHT \
            else get_prev_orientation(orientation)

    def turn_to(target_orientation):
        global robot_state, orientation, ongoing_motion, target_yaw, target_direction
        ongoing_motion += 1
        target_yaw = round((pi - (pi / 4 * target_orientation.value) + pi / 4), 2)

        prev_orientartion = orientation
        turn_to_target = prev_orientartion.value - target_orientation.value

        if turn_to_target == 0:
            target_direction = -1
        elif turn_to_target < -4 or (turn_to_target > 0 and turn_to_target < 4):
            target_direction = Direction.LEFT
        else:
            target_direction = Direction.RIGHT

        next_state(State.TURN)
        orientation = target_orientation

    def append_and_update(target):
        global state_queue, target_orientation

        state_queue.append(('t', target))
        target_orientation = target

    def append_state():
        global state_queue, current_x, current_y
        target_node_x, target_node_y = route.pop(0)

        dx = target_node_x - current_x
        dy = target_node_y - current_y

        current_x = target_node_x
        current_y = target_node_y

        if (dy > 0 and dx > 0):
            append_and_update(Orientation.NORTH_EAST)
        elif (dy > 0 and dx < 0):
            append_and_update(Orientation.NORTH_WEST)
        elif (dy < 0 and dx > 0):
            append_and_update(Orientation.SOUTH_EAST)
        elif (dy < 0 and dx < 0):
            append_and_update(Orientation.SOUTH_WEST)
        elif (dy > 0):
            append_and_update(Orientation.NORTH)
        elif (dy < 0):
            append_and_update(Orientation.SOUTH)
        elif (dx > 0):
            append_and_update(Orientation.EAST)
        elif (dx < 0):
            append_and_update(Orientation.WEST)

        state_queue.append(('w', None))
    
    for i in range (0, len(route)):
        append_state()
    
    print(state_queue)
    key, arg = state_queue.pop(0)
    orientation = Orientation.NORTH
    target_orientation = Orientation.NORTH
    target_direction = None
    current_x = 0
    current_y = 0
    ongoing_motion = 0
    target_yaw = imu.getRollPitchYaw()[2]

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(TIMESTEP) != -1:
        
        if key == 'w' and current_state == State.IDLE and ongoing_motion == 0:
            next_state(State.MOVE_FORWARD)
        
        if key == 't' and current_state != State.TURN and ongoing_motion == 0:
            turn_to(arg)

        current_time = robot.getTime()

        left_speed = 0
        right_speed = 0

        if current_state == State.IDLE:
            target_direction = None
            left_speed = 0
            right_speed = 0
            ongoing_motion -= 1
            
            if len(state_queue) <= 0:
                break
            key, arg = state_queue.pop(0)

        elif current_state == State.TURN:
            yaw = imu.getRollPitchYaw()[2]
            delta_yaw = target_yaw - yaw

            # print(target_yaw, yaw, delta_yaw, target_direction, orientation)

            if abs(delta_yaw) <= 0.05:
                left_speed = 0
                right_speed = 0
                next_state(State.IDLE)
            else:
                if abs(delta_yaw) < 0.25 and target_direction == -1:
                    if delta_yaw < 0:
                        target_direction = Direction.RIGHT
                    else:
                        target_direction = Direction.LEFT
                
                if target_direction == Direction.LEFT:
                    left_speed = -0.5 * max_speed
                    right_speed = 0.5 * max_speed
                else:
                    left_speed = 0.5 * max_speed
                    right_speed = -0.5 * max_speed

        elif current_state == State.MOVE_FORWARD:
            if initial:
                ongoing_motion += 1
                x, y, z = gps.getValues()
                x = round(x, 3)
                y = round(y, 3)
                prev_position = (x, y)

                if orientation == Orientation.NORTH:
                    current_y += 1
                elif orientation == Orientation.NORTH_EAST:
                    current_x += 1
                    current_y += 1
                elif orientation == Orientation.EAST:
                    current_x += 1
                elif orientation == Orientation.SOUTH_EAST:
                    current_x += 1
                    current_y -= 1
                elif orientation == Orientation.SOUTH:
                    current_y -= 1
                elif orientation == Orientation.SOUTH_WEST:
                    current_x -= 1
                    current_y -= 1
                elif orientation == Orientation.WEST:
                    current_x -= 1
                elif orientation == Orientation.NORTH_WEST:
                    current_x -= 1
                    current_y += 1

                initial = False

            x, y, z = gps.getValues()
            x = round(x, 3)
            y = round(y, 3)
            prev_x, prev_y = prev_position

            target_x = ((current_x) * TILE_SIZE) + (TILE_SIZE / 2)
            target_y = ((current_y) * TILE_SIZE) + (TILE_SIZE / 2)
            dx = abs(x - target_x)
            dy = abs(y - target_y)

            right_diagonal_orientations = [Orientation.NORTH_EAST, Orientation.SOUTH_EAST]
            left_diagonal_orientations = [Orientation.NORTH_WEST, Orientation.SOUTH_WEST]
            diagonal_orientations = right_diagonal_orientations + left_diagonal_orientations

            x_orientations = diagonal_orientations + [Orientation.EAST, Orientation.WEST]
            y_orientations = diagonal_orientations + [Orientation.NORTH, Orientation.SOUTH]

            reached_x = (orientation in x_orientations) and (dx <= 0.005)
            reached_y = (orientation in y_orientations) and (dy <= 0.005)

            if ((orientation not in diagonal_orientations and (reached_x or reached_y)) \
                or (dx + dy <= 0.03)):
                left_speed = 0
                right_speed = 0
                next_state(State.IDLE)
            else:
                if orientation in diagonal_orientations:
                    left_speed = max_speed * 0.75
                    right_speed = max_speed * 0.75
                else:
                    left_speed = max_speed
                    right_speed = max_speed

                if orientation == Orientation.NORTH:
                    if x < target_x:
                        left_speed += A_COMPENSATION
                    elif x > target_x: 
                        right_speed += A_COMPENSATION
                elif orientation == Orientation.EAST:
                    if y > target_y:
                        left_speed += A_COMPENSATION
                    elif y < target_y:
                        right_speed += A_COMPENSATION
                elif orientation == Orientation.SOUTH:
                    if x > target_x:
                        left_speed += A_COMPENSATION
                    elif x < target_x:
                        right_speed += A_COMPENSATION
                elif orientation == Orientation.WEST:
                    if y < target_y:
                        left_speed += A_COMPENSATION
                    elif y > target_y:
                        right_speed += A_COMPENSATION
                elif orientation == Orientation.SOUTH_EAST or orientation == Orientation.NORTH_WEST:
                    if dy > dx:
                        left_speed += A_COMPENSATION * 0.75
                    elif dy < dx:
                        right_speed += A_COMPENSATION * 0.75
                elif orientation == Orientation.NORTH_EAST or orientation == Orientation.SOUTH_WEST:
                    if dy < dx:
                        left_speed += A_COMPENSATION * 0.75
                    elif dy > dx:
                        right_speed += A_COMPENSATION * 0.75

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
# Enter here exit cleanup code.
