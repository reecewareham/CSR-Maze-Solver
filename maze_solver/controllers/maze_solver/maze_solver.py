"""maze_solver controller."""

from controller import Robot, Motor, DistanceSensor, Camera
import math

robot = Robot()

timestep = int(robot.getBasicTimeStep())

# Ground Sensor Setup.

gs = []
gsNames = ["gs0", "gs1", "gs2"]

for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# Proximity Sensor Setup.

ps = []
psNames = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)
    
# Motor Setup.

MAX_SPEED = 6.28

speedValues = [MAX_SPEED, MAX_SPEED]

leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Camera Setup.

camera = robot.getDevice("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)

# GPS Setup.

gps = robot.getDevice("gps")
gps.enable(timestep)

# Main Loop.

while robot.step(timestep) != -1:

    # Get list of objects recognised by camera.
    objects = camera.getRecognitionObjects()
    
    # Get numbers of objects recognised by camera.
    noOfObjects = camera.getRecognitionNumberOfObjects()
    
    # Checks the number of objects.
    if noOfObjects > 0:
        
        # Get position of the first object.
        position = objects[0].getPosition()
        
        #  Checks how close the object is and prints the rough GPS coordinates of the object.
        if position[0] < 0.1:
            gpsValues = gps.getValues()
            print(f"Object detected at: {gpsValues}")

    # Get the values of all the proximity sensors.
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
        #print("Sensor: {}  Value: {}".format(i, psValues[i]))   - Debug Code
    
    # Get the values of all the ground sensors.
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())
        #print("Sensor: {}  Value: {}".format(i, gsValues[i]))   - Debug Code
    
    # Checks how close the robot is to the walls using the proximity sensors.
    # Returns either true or false.
    leftWall = psValues[5] > 80
    frontWall = psValues[7] > 80 or psValues[0] > 80
    leftCorner = psValues[6] > 80
    leftWallTooClose = psValues[5] > 200
    
    
    # Checks the ground sensors and if there is a front wall, stop as robot has reached the goal.
    # If there is no wall, continue forward.
    if int(gsValues[0]) & int(gsValues[1]) & int(gsValues[2]) <= 450:
        if frontWall:
            #print("Reached Goal")   - Debug Code
            speedValues[0] = 0
            speedValues[1] = 0
        else:
            speedValues[0] = MAX_SPEED
            speedValues[1] = MAX_SPEED
    else:
        # If robot detects front wall, turn right.
        if frontWall:
            #print("Front wall detected! Turning right.")   - Debug Code
            speedValues[0] = MAX_SPEED
            speedValues[1] = -MAX_SPEED
            
        # If robot is too close to left wall, turn right a little.    
        elif leftWallTooClose:
            #print("Left wall too close! Turning right.")   - Debug Code
            speedValues[0] = MAX_SPEED
            speedValues[1] = MAX_SPEED / 2
            
        # If robot detects left wall, continue forward.    
        elif leftWall:
            #print("Left wall detected! Following wall.")   - Debug Code
            speedValues[0] = MAX_SPEED
            speedValues[1] = MAX_SPEED
            
        # If robot detects a corner, turn right.    
        elif leftCorner:
            #print("Left corner detected! Turning right.")   - Debug Code
            speedValues[0] = MAX_SPEED
            speedValues[1] = MAX_SPEED / 16
            
        # If robot doesn't detect any walls, turn left towards the left wall.     
        else:
            #print("No wall detected! Turning left to find wall.")   - Debug Code
            speedValues[0] = MAX_SPEED / 4
            speedValues[1] = MAX_SPEED
        
    # Sets the speed of the motors.
    leftMotor.setVelocity(speedValues[0])
    rightMotor.setVelocity(speedValues[1])
    
    pass
