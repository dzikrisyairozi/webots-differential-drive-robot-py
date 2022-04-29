"""differential-drive-robot-controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from a_star import get_route
from controller import Robot as WebotsRobot, GPS, Keyboard
from enums import Direction, State, Compass

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

    #Motor Instances
    left_motor = robot.getDevice('motor_1')
    right_motor = robot.getDevice('motor_2')

    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    num_side = 4
    length_side = 0.25

    wheel_radius = 0.025
    linear_velocity = wheel_radius * max_speed

    duration_side = length_side / linear_velocity

    angle_of_rotation = 6.28 / num_side
    distance_between_wheels = 0.090
    rate_of_rotation = (2 * linear_velocity) / distance_between_wheels
    duration_turn = angle_of_rotation / rate_of_rotation

    rot_start_time = -1
    rot_end_time = -1

    turn_side = None

    x, y, z = gps.getValues()
    prev_position = (x, y)

    orientation = Compass.NORTH
    A_COMPENSATION = 1

    TILE_SIZE = 0.25

    current_state = State.IDLE
    prev_state = None
    initial = False

    route = get_route((0, 0), (7, 7))
    current_x, current_y = route.pop(0)
    state_queue = []

    def next_state(next_state):
        global current_state, prev_state, initial
        prev_state = current_state
        current_state = next_state

        initial = True
    
    def update_orientation(direction):
        global orientation
        if direction == Direction.RIGHT:
            if orientation == Compass.NORTH:
                orientation = Compass.EAST
            elif orientation == Compass.EAST:
                orientation = Compass.SOUTH
            elif orientation == Compass.SOUTH:
                orientation = Compass.WEST
            elif orientation == Compass.WEST:
                orientation = Compass.NORTH
        elif direction == Direction.LEFT:
            if orientation == Compass.NORTH:
                orientation = Compass.WEST
            elif orientation == Compass.WEST:
                orientation = Compass.SOUTH
            elif orientation == Compass.SOUTH:
                orientation = Compass.EAST
            elif orientation == Compass.EAST:
                orientation = Compass.NORTH

    def turn(direction):
        global rot_start_time, rot_end_time, turn_side, robot_state, orientation, ongoing_motion
        
        ongoing_motion += 1
        rot_start_time = current_time + duration_side
        rot_end_time = rot_start_time + duration_turn
        turn_side = direction
        next_state(State.TURN)
        update_orientation(direction)

    
    def append_and_update(direction):
        global state_queue
        state_queue.append(direction)
        if direction == 'd':
            update_orientation(Direction.RIGHT)
        elif direction == 'a':
            update_orientation(Direction.LEFT)

    def append_state():
        global state_queue, current_x, current_y
        target_node_x, target_node_y = route.pop(0)

        dx = target_node_x - current_x
        dy = target_node_y - current_y

        # print("dy: ", dy)

        current_x = target_node_x
        current_y = target_node_y

        if orientation == Compass.NORTH and dy < 0:
            append_and_update('d')
            append_and_update('d')
        elif orientation == Compass.NORTH and dx > 0:
            append_and_update('d')
        elif orientation == Compass.NORTH and dx < 0:
            append_and_update('a')
        elif orientation == Compass.SOUTH and dy > 0:
            append_and_update('d')
            append_and_update('d')
        elif orientation == Compass.SOUTH and dx > 0:
            append_and_update('a')
        elif orientation == Compass.SOUTH and dx < 0:
            append_and_update('d')
        elif orientation == Compass.EAST and dy > 0:
            append_and_update('a')
        elif orientation == Compass.EAST and dy < 0:
            append_and_update('d')
        elif orientation == Compass.EAST and dx < 0:
            append_and_update('d')
            append_and_update('d')
        elif orientation == Compass.WEST and dy > 0:
            append_and_update('d')
        elif orientation == Compass.WEST and dy < 0:
            append_and_update('a')
        elif orientation == Compass.WEST and dx > 0:
            append_and_update('d')
            append_and_update('d')

        state_queue.append('w')
    
    for i in range (0, len(route)):
        append_state()
    
    # print(state_queue)
    key = state_queue.pop(0)
    orientation = Compass.NORTH
    current_x = 0
    current_y = 0
    ongoing_motion = 0

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(TIMESTEP) != -1:

        if key == 'w' and current_state == State.IDLE and ongoing_motion == 0:
            next_state(State.MOVE_FORWARD)
            
        if key == 'a' and current_state != State.TURN and ongoing_motion == 0:
            turn(Direction.LEFT)

        if key == 'd' and current_state != State.TURN and ongoing_motion == 0:
            turn(Direction.RIGHT)

        current_time = robot.getTime()

        left_speed = 0
        right_speed = 0

        if current_state == State.IDLE:
            left_speed = 0
            right_speed = 0
            ongoing_motion -= 1
            
            if len(state_queue) <= 0:
                break
            key = state_queue.pop(0)

        elif current_state == State.TURN:
            if rot_start_time < current_time < rot_end_time:
                if turn_side == Direction.LEFT:
                    left_speed = -max_speed
                    right_speed = max_speed
                else:
                    left_speed = max_speed
                    right_speed = -max_speed

            elif current_time >= rot_end_time:
                left_speed = 0
                right_speed = 0
                next_state(State.IDLE)

        elif current_state == State.MOVE_FORWARD:
            if initial:
                ongoing_motion += 1
                x, y, z = gps.getValues()
                x = round(x, 3)
                y = round(y, 3)
                prev_position = (x, y)

                if orientation == Compass.NORTH:
                    current_y += 1
                elif orientation == Compass.EAST:
                    current_x += 1
                elif orientation == Compass.SOUTH:
                    current_y -= 1
                elif orientation == Compass.WEST:
                    current_x -= 1

                initial = False

            x, y, z = gps.getValues()
            x = round(x, 3)
            y = round(y, 3)
            prev_x, prev_y = prev_position

            target_x = ((current_x) * TILE_SIZE) + (TILE_SIZE / 2)
            target_y = ((current_y) * TILE_SIZE) + (TILE_SIZE / 2)
            dx = abs(x - target_x)
            dy = abs(y - target_y)

            reached_x = (orientation == Compass.EAST or orientation == Compass.WEST) and (dx <= 0.005)
            reached_y = (orientation == Compass.NORTH or orientation == Compass.SOUTH) and (dy <= 0.005)

            if (reached_x or reached_y):
                left_speed = 0
                right_speed = 0
                next_state(State.IDLE)
            else:
                left_speed = max_speed
                right_speed = max_speed

                if orientation == Compass.NORTH:
                    if x < target_x:
                        left_speed += A_COMPENSATION
                    elif x > target_x: 
                        right_speed += A_COMPENSATION
                elif orientation == Compass.EAST:
                    if y > target_y:
                        left_speed += A_COMPENSATION
                    elif y < target_y:
                        right_speed += A_COMPENSATION
                elif orientation == Compass.SOUTH:
                    if x > target_x:
                        left_speed += A_COMPENSATION
                    elif x < target_x:
                        right_speed += A_COMPENSATION
                elif orientation == Compass.WEST:
                    if y < target_y:
                        left_speed += A_COMPENSATION
                    elif y > target_y:
                        right_speed += A_COMPENSATION

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    # Enter here exit cleanup code.
