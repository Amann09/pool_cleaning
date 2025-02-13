"""robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    # timestep = int(robot.getBasicTimeStep())
    timestep = 64
    max_speed = 6.28  # angular_velocity
    
    # Created Motor Instances
    front_right_motor = robot.getMotor('front_motor_1')
    front_left_motor = robot.getMotor('front_motor_2')
    
    back_right_motor = robot.getMotor('back_motor_1')
    back_left_motor = robot.getMotor('back_motor_2')
    
    
    front_right_motor.setPosition(float('inf'))
    front_right_motor.setVelocity(0.0)
    
    front_left_motor.setPosition(float('inf'))
    front_left_motor.setVelocity(0.0)
    
    
    back_right_motor.setPosition(float('inf'))
    back_right_motor.setVelocity(0.0)
    
    back_left_motor.setPosition(float('inf'))
    back_left_motor.setVelocity(0.0)
    
    # Driving the Robot in a Polygon Shape
    num_side = 4
    length_side = 0.5  # 0.25
    # rr = 1
    
    # wheel_radius = 0.025
    # linear_velocity = wheel_radius * max_speed
    
    # duration_side = length_side / linear_velocity
    
    # start_time = robot.getTime()
    
    # angle_of_rotation = 6.28/num_side
    # distance_between_wheels = 0.156
    # rate_of_rotation = (rr * linear_velocity) / distance_between_wheels 
    # duration_turn = angle_of_rotation / rate_of_rotation 
    
    # rot_start_time = start_time + duration_side
    # rot_end_time = rot_start_time + duration_turn
    
    front_right_distance_sensor = robot.getDevice("distance_sensor_front_right")
    front_left_distance_sensor = robot.getDevice("distance_sensor_front_left")
    
    front_right_distance_sensor.enable(timestep)
    front_left_distance_sensor.enable(timestep)
    
    
    right_1_distance_sensor = robot.getDevice("distance_sensor_right_1")
    left_1_distance_sensor = robot.getDevice("distance_sensor_left_1")
    
    right_1_distance_sensor.enable(timestep)
    left_1_distance_sensor.enable(timestep)
    
    
    right_2_distance_sensor = robot.getDevice("distance_sensor_right_2")
    left_2_distance_sensor = robot.getDevice("distance_sensor_left_2")
    
    right_2_distance_sensor.enable(timestep)
    left_2_distance_sensor.enable(timestep)
   
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
    
        # current_time = robot.getTime()
    
        # left_speed = max_speed
        # right_speed = max_speed
        
        # if rot_start_time < current_time < rot_end_time:
            # left_speed = -max_speed
            # right_speed = max_speed
            
        # elif current_time > rot_end_time:
            # rot_start_time = current_time + duration_side
            # rot_end_time = rot_start_time + duration_turn
        
        
        
        
        right_front_value = front_right_distance_sensor.getValue()
        print("Front Right Sensor value is: ", right_front_value)
        
        left_front_value = front_left_distance_sensor.getValue()
        print("Front Left Sensor value is: ", left_front_value)
        
        
        right_1_value = right_1_distance_sensor.getValue()
        # print("Right 1 Value:", right_1_value)
        
        left_1_value = left_1_distance_sensor.getValue()
        # print("Right 1 Value:", left_1_value)
        
        right_2_value = right_2_distance_sensor.getValue()
        # print("Right 2 Value:", right_2_value)
        
        left_2_value = left_2_distance_sensor.getValue()
        # print("Right 1 Value:", left_2_value)
        
        
        
        if (right_front_value < 990) and (left_front_value < 990):
            left_speed = -max_speed
            right_speed = max_speed 
        elif right_front_value > 990:
            left_speed = max_speed
            right_speed = max_speed
        
            
        # if right_1_value < 900 and right_2_value < 900:
            # left_speed = 0
            # right_speed = 0
        
        
        front_right_motor.setVelocity(right_speed)
        front_left_motor.setVelocity(left_speed)
        
        back_right_motor.setVelocity(right_speed)
        back_left_motor.setVelocity(left_speed)
            
        
        
        
    # Enter here exit cleanup code.