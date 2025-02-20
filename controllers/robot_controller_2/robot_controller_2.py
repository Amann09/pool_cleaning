"""robot_controller_2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from math import pi
from enum import Enum, auto
import random

class Turn(Enum):
    CLOCKWISE = auto()
    ANTI_CLOCKWISE = auto()

# create the Robot instance.
robot = Robot()
pi = round(pi, 3)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep()) # timestep = 32
max_speed = 6.28

safe_distance = 0.35 # in meters

angles = [30, 45, 60, 90, 180]

# Motors
front_right_motor = robot.getMotor('front_motor_1')
front_left_motor = robot.getMotor('front_motor_2')

front_right_motor.setPosition(float('inf'))
front_right_motor.setVelocity(0.0)
    
front_left_motor.setPosition(float('inf'))
front_left_motor.setVelocity(0.0)


back_right_motor = robot.getMotor('back_motor_1')
back_left_motor = robot.getMotor('back_motor_2')
    
back_right_motor.setPosition(float('inf'))
back_right_motor.setVelocity(0.0)
    
back_left_motor.setPosition(float('inf'))
back_left_motor.setVelocity(0.0)


# Distance Sensors
front_distance_sensor = robot.getDevice("distance_sensor_front")
side_right_distance_sensor = robot.getDevice("distance_sensor_side_right")
side_left_distance_sensor = robot.getDevice("distance_sensor_side_left")

front_distance_sensor.enable(timestep)
side_right_distance_sensor.enable(timestep)
side_left_distance_sensor.enable(timestep)

# Inertial Unit
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

def rotate(theta):
    if theta > 0:
        turn = Turn.CLOCKWISE
    else:
        turn = Turn.ANTI_CLOCKWISE
    
    START_THETA = (imu.getRollPitchYaw()[2] * (180/pi)) % 360
    endpointRIGHT = (START_THETA + theta) % 360
    endpointLEFT = (START_THETA - theta) % 360
    
    print(f"Rotating the vehicle {turn}")
    print(' ')


    if turn == Turn.CLOCKWISE:
        endpointRIGHT = (START_THETA - theta) % 360
        endpointLEFT = (START_THETA + theta) % 360
        
        while robot.step(timestep) != -1:
            front_value = front_distance_sensor.getValue()
            side_right_value = side_right_distance_sensor.getValue()
            side_left_value = side_left_distance_sensor.getValue()
            
            print(f"Rotate\n(Front, Right, Left) -> ({front_value}, {side_right_value}, {side_left_value})")
            
            reading = (imu.getRollPitchYaw()[2] * (180/pi)) % 360
            print(f"\n(START_THETA, Reading, endPointRight, endPointLeft) -> ({START_THETA}, {reading}, {endpointRIGHT}, {endpointLEFT})")
            
            print(f' [turning right {theta} degrees]')
            front_left_motor.setVelocity(max_speed)
            front_right_motor.setVelocity(-max_speed)
    
            back_left_motor.setVelocity(max_speed)
            back_right_motor.setVelocity(-max_speed)
            
            if 0 <= abs(reading - endpointRIGHT) <= 10:
            # if reading < endpointRIGHT:
                print("Turning Right complete")
                forward()
                    
    if turn == Turn.ANTI_CLOCKWISE:
        endpointRIGHT = (START_THETA + theta) % 360
        endpointLEFT = (START_THETA - theta) % 360

        while robot.step(timestep) != -1:
            front_value = front_distance_sensor.getValue()
            side_right_value = side_right_distance_sensor.getValue()
            side_left_value = side_left_distance_sensor.getValue()
            
            print(f"Rotate\n(Front, Right, Left) -> ({front_value}, {side_right_value}, {side_left_value})")
            
            reading = (imu.getRollPitchYaw()[2] * (180/pi)) % 360
            print(f"\n(START_THETA, Reading, endPointRight, endPointLeft) -> ({START_THETA}, {reading}, {endpointRIGHT}, {endpointLEFT})")
            
            print(f' [turning left {theta} degrees]')
            front_left_motor.setVelocity(-max_speed)
            front_right_motor.setVelocity(max_speed)
    
            back_left_motor.setVelocity(-max_speed)
            back_right_motor.setVelocity(max_speed)  
            
            # if reading > endpointLEFT:
                # print("Turning Left complete")
                # forward()

            if 0 <= abs(reading - endpointLEFT) <= 10:
               print("Turning Left complete")
               forward()
def stop():

    left_speed = 0
    right_speed = 0
    
    front_right_motor.setVelocity(right_speed)
    front_left_motor.setVelocity(left_speed)
        
    back_right_motor.setVelocity(right_speed)
    back_left_motor.setVelocity(left_speed)


def forward():
    while robot.step(timestep) != -1:
        front_value = front_distance_sensor.getValue()
        side_right_value = side_right_distance_sensor.getValue()
        side_left_value = side_left_distance_sensor.getValue()
        
        print(f"Forward\n(Front, Right, Left) -> ({front_value}, {side_right_value}, {side_left_value})")
        # print("----------------------------------------------------------------------------------")
        # print(f"IMU Roll Pitch Yaw: {imu.getRollPitchYaw()}")
        print(' ')

        left_speed = max_speed
        right_speed = max_speed
        
        if front_value < safe_distance:
            angles = [15, 30, 45, 60, 75, 90]
            random.shuffle(angles)
            for angle in angles:
                # x = random.choice([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
                # if x%2 == 0:
                    # rotate(angle)
                # else:
                    # rotate(-angle)
                    
                if side_right_value < safe_distance-0.1:
                    print("Turning Left")
                    rotate(-angle)
                elif side_left_value < safe_distance-0.1:
                    print("Turning Right")
                    rotate(angle)
                else:
                    x = random.choice([-1, 1])
                    print("Turning {x}")
                    rotate(x * angle)
                    
                # rotate(angle) 


            
        if side_left_value < safe_distance - 0.1:
            print('--->  LEFT WALL DETECTED ' + str(side_left_value) + ' [turning slight right...]')
            left_speed = max_speed
            right_speed = max_speed - .20
        elif side_right_value < safe_distance - 0.1:
            print('--->  RIGHT WALL DETECTED ' + str(side_right_value) + ' [turning slight left...]')
            left_speed = max_speed - .20
            right_speed = max_speed
        else:
            if left_speed > max_speed:
                print('TOO HIGH => setting velocity to 6.28')
                left_speed = max_speed
            elif left_speed < -max_speed:
                print('TOO LOW => setting velocity to -6.28')
                left_speed = -max_speed  
            else: 
                left_speed = left_speed
                right_speed = left_speed

        front_right_motor.setVelocity(right_speed)
        front_left_motor.setVelocity(left_speed)
        
        back_right_motor.setVelocity(right_speed)
        back_left_motor.setVelocity(left_speed)

    

# Main loop:
while robot.step(timestep) != -1:
    forward()
    