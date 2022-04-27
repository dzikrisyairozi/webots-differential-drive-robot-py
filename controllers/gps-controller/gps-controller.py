"""differential-drive-robot-controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot as WebotsRobot, GPS, Keyboard
from motion import Direction, State

if __name__ == "__main__":

    # create the Robot instance.
    robot = WebotsRobot()

    # get the time step of the current world.
    timestep = 64
    max_speed = 6.28 #angular velocity

    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    gps = GPS('gps')
    gps.enable(64)

    keyboard = Keyboard()
    keyboard.enable(64)

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
    linear_velocity = wheel_radius *  max_speed

    duration_side = length_side/linear_velocity

    start_time = robot.getTime()

    angle_of_rotation = 6.28/num_side
    distance_between_wheels = 0.090
    rate_of_rotation = (2 * linear_velocity)/ distance_between_wheels
    duration_turn = angle_of_rotation/rate_of_rotation

    # 0 + duration_side => drive straight
    # > duration_side till duration_turn => turn

    # duration_side > and < duration turn => turn

    rot_start_time = -1
    rot_end_time = -1

    turn_side = None

    robot_state = State.IDLE

    x, y, z = gps.getValues()
    prev_position = (x, y)

    def turn(direction):
        global rot_start_time, rot_end_time, turn_side, robot_state
        
        rot_start_time = current_time + duration_side
        rot_end_time = rot_start_time + duration_turn
        turn_side = direction
        robot_state = State.TURN

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        current_time = robot.getTime()

        # left_speed = 0.5 * max_speed
        # right_speed = 0.5 * max_speed
        left_speed = 0
        right_speed = 0

        key = keyboard.getKey()
        if key == ord('A'):
            turn(Direction.LEFT)

        if key == ord('D'):
            turn(Direction.RIGHT)

        if key == ord('W') and robot_state == State.IDLE:
            x, y, z = gps.getValues()
            x = round(x, 3)
            y = round(y, 3)
            prev_position = (x, y)
            
            robot_state = State.MOVE_FORWARD

        if key == ord('S'):
            robot_state = State.IDLE
        
        if robot_state == State.IDLE:
            left_speed = 0
            right_speed = 0

        elif robot_state == State.TURN:
            if rot_start_time < current_time < rot_end_time:
                print(rot_start_time, current_time, rot_end_time)
                if turn_side == Direction.LEFT:
                    left_speed = -max_speed
                    right_speed = max_speed
                else:
                    left_speed = max_speed
                    right_speed = -max_speed
            elif current_time >= rot_end_time:
                left_speed = 0
                right_speed = 0
                robot_state = State.IDLE

        elif robot_state == State.MOVE_FORWARD:
            x, y, z = gps.getValues()
            x = round(x, 3)
            y = round(y, 3)
            prev_x, prev_y = prev_position

            print(abs(y - prev_y))

            if (abs(x - prev_x) >= 0.25 or abs(y - prev_y) >= 0.25):
                left_speed = 0
                right_speed = 0
                robot_state = State.IDLE
            else:
                left_speed = max_speed
                right_speed = max_speed

        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

    # Enter here exit cleanup code.