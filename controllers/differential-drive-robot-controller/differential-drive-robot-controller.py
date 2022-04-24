"""differential-drive-robot-controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

if __name__ == "__main__":

    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = 64
    max_speed = 6.28 #angular velocity
    
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    
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
    
    rot_start_time = start_time + duration_side
    rot_end_time = rot_start_time + duration_turn
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
    
        current_time = robot.getTime()
        
        left_speed = 0.5 * max_speed
        right_speed = 0.5 * max_speed
        
        if rot_start_time < current_time < rot_end_time:
            left_speed = -max_speed
            right_speed = max_speed
            
        elif current_time > rot_end_time:
            rot_start_time = current_time + duration_side
            rot_end_time = rot_start_time + duration_turn
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    
    # Enter here exit cleanup code.
    