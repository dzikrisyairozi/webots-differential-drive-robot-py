"""odometer_calculation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

def run_robot(robot):
    # get the time step of the current world.
    timestep = 64
    max_speed = 6.28
    
    #Motor Instances
    left_motor = robot.getDevice('motor_1')
    right_motor = robot.getDevice('motor_2')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    #Posisiton sensor instances
    left_ps = robot.getDevice('ps_1')
    left_ps.enable(timestep)
    
    right_ps = robot.getDevice('ps_2')
    right_ps.enable(timestep)
    
    ps_values = [0, 0]
    dist_values = [0, 0]
    
    #compute encoder unit
    wheel_radius = 0.025
    distance_between_wheels = 0.09
    
    wheel_circum = 2 * 3.14 * wheel_radius
    encoder_unit = wheel_circum/6.28
    
    # robot pose
    robot_pose = [0, 0, 0] #x, y, theta
    last_ps_values = [0, 0]

    num_side = 4
    length_side = 0.5
    
    wheel_radius = 0.025
    linear_velocity = wheel_radius *  max_speed
    
    duration_side = length_side/linear_velocity
    
    start_time = robot.getTime()
    
    angle_of_rotation = 6.28/num_side
    distance_between_wheels = 0.090
    rate_of_rotation = (2 * linear_velocity)/ distance_between_wheels
    duration_turn = angle_of_rotation/rate_of_rotation
    
    rot_start_time = start_time + duration_side
    rot_end_time = rot_start_time + duration_turn
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read values from position sensor
        ps_values[0] = left_ps.getValue()
        ps_values[1] = right_ps.getValue()
        
        print("=====================")
        print("position sensor values: {} {}".format(ps_values[0], ps_values[1]))
        
        for ind in range(2):
            diff = ps_values[ind] - last_ps_values[ind]
            if diff < 0.001:
                diff = 0
                ps_values[ind] = last_ps_values[ind]
            dist_values[ind] = diff * encoder_unit
            
        #compute linear and angular velocity for robot
        v = (dist_values[0] + dist_values[1])/2.0
        w = (dist_values[0] - dist_values[1])/distance_between_wheels
        
        dt = 1
        robot_pose[2] += (w * dt)
        
        vx = v * math.cos(robot_pose[2])
        vy = v * math.sin(robot_pose[2])
        
        robot_pose[0] += (vx * dt)
        robot_pose[1] += (vy * dt)
        
        print("robot_pose: {}".format(robot_pose))
            
        #print("distance values: {} {}".format(dist_values[0], dist_values[1]))    

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
        
        for ind in range(2):
            last_ps_values[ind] = ps_values[ind]

if __name__ == "__main__":

    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)
    
