"""odometer_calculation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

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
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read values from position sensor
        ps_values[0] = left_ps.getValue()
        ps_values[1] = right_ps.getValue()
        
        print("=====================")
        print("position sensor values: {} {}".format(ps_values[0], ps_values[1]))
        
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(max_speed)

    

if __name__ == "__main__":

    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)
    
