
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor

import math
import numpy as np

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

def toEuler(x, y, z, angle):
    s = math.sin(angle)
    c = math.cos(angle)
    t = 1 - c

    magnitude = math.sqrt(x*x + y*y + z*z)
    x /= magnitude
    y /= magnitude
    z /= magnitude
    if ((x*y*t + z*s) > 0.998): # north pole singularity detected
        heading = 2*math.atan2(x*math.sin(angle/2), math.cos(angle/2))
        attitude = math.pi/2
        bank = 0
    elif ((x*y*t + z*s) < -0.998): # south pole singularity detected
        heading = -2*math.atan2(x*math.sin(angle/2),math.cos(angle/2))
        attitude = -math.pi/2
        bank = 0
    else:
        heading = math.atan2(y * s- x * z * t , 1 - (y*y+ z*z ) * t)
        attitude = math.asin(x * y * t + z * s)
        bank = math.atan2(x * s - y * z * t , 1 - (x*x + z*z) * t)

    return [heading, attitude, bank]

def rotate(heading, attitude, bank):
    # Assuming the angles are in radians.
    c1 = math.cos(heading/2)
    s1 = math.sin(heading/2)
    c2 = math.cos(attitude/2)
    s2 = math.sin(attitude/2)
    c3 = math.cos(bank/2)
    s3 = math.sin(bank/2)
    c1c2 = c1*c2
    s1s2 = s1*s2
    w =c1c2*c3 - s1s2*s3
    x =c1c2*s3 + s1s2*c3
    y =s1*c2*c3 + c1*s2*s3
    z =c1*s2*c3 - s1*c2*s3
    angle = 2 * math.acos(w)
    norm = x*x+y*y+z*z
    if (norm < 0.001): # when all euler angles are zero angle =0 so
        # we can set axis to anything to avoid divide by zero
        x=1
        y=0
        z=0
    else:
        norm = math.sqrt(norm)
        x /= norm
        y /= norm
        z /= norm
    
    return [x, y, z, angle]


red1 = robot.getFromDef("red1")
red1translation = red1.getField("translation")
red1rotation = red1.getField("rotation")

red2 = robot.getFromDef("red2")
red2translation = red2.getField("translation")
red2rotation = red2.getField("rotation")

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    tra = red1translation.getSFVec3f()
    rot = red1rotation.getSFRotation()
    euler = toEuler(rot[0], rot[1], rot[2], rot[3])
    rotationout = rotate(euler[0], math.pi-euler[1], euler[2])
    red2translation.setSFVec3f([-tra[0], tra[1], tra[2]])
    red2rotation.setSFRotation(rotationout)
    # print("tra:", tra)
    # print("rot:", rot)
    # print(np.array(euler)*180/math.pi)
    # print(rotate(euler[0], euler[1], euler[2]))
    # print("------------------------------------------")
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
