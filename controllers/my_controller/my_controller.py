"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
import numpy as np
import math
from controller import Supervisor
from controller import TouchSensor
try:
    import ikpy
    from ikpy.chain import Chain
    from ikpy.link import URDFLink, OriginLink
except ImportError:
    sys.exit('The "ikpy" has not been installed. Please install the newest version')
    
if ikpy.__version__[0] < '3':
    sys.exit('Please install the version of "ikpy" 3.#')

# activating chain from urdf
armChain = Chain.from_urdf_file('ur5.urdf')

# create the Supervisor instance.
supervisor = Supervisor()
# get the time step of the current world.
timestep = int(16 * supervisor.getBasicTimeStep())

#initial position of arm
initialPosition = [1.57, -0.785, 0.785, 1.57, 1.57, 0]

# navigation of the robot 
def navigation(initialPosition, sensors):
    
    change = 0
    linearOpen = False
    angle1Change = 0
    angle2Change = 0
    angles = []
    
    
    if avrFromSensors(sensors, len(sensors)) < (10/6):
        change = 0.1 * math.pi
        linearOpen = True
        angles.append(linearOpen)
        angle1Change = -(initialPosition[1] - change)
        angle2Change = -(initialPosition[2] - change)
    elif avrFromSensors(sensors, len(sensors)) >= (10/6):
        angle1Change = initialPosition[1]
        angle2Change = initialPosition[2]
        angles.append(linearOpen)
    
    
       
    angles.append(angle1Change)
    angles.append(angle2Change)
    angles.append(initialPosition[3] - angle1Change + angle2Change)
     
    return angles    

def avrFromSensors(sensors, numOfSensors):
    sumOfAllSensors = 0
    for i in range(len(sensors)):
        sumOfAllSensors = sumOfAllSensors + sensors[i].getValue()
    avr = sumOfAllSensors / numOfSensors
    return avr

# initialising the motors of the arm
motors = []
for motorName in ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']:
    motor = supervisor.getMotor(motorName)
    motor.setVelocity(0.4)
    motors.append(motor)
# initialising the motor of the gripper
linear = supervisor.getMotor('linear motor')
 
# initialising the touch sensors
sensors = []
for sensorName in ['ts1','ts2','ts3','ts4','ts5','ts6']:
    sensor = supervisor.getTouchSensor(sensorName)
    TouchSensor.enable(sensor, timestep)
    sensors.append(sensor)
    

# testing target coordinates for now
arm = supervisor.getSelf()
gripper = supervisor.getFromDef('BaseOfGripper')

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1:
    t = supervisor.getTime()
    
    # Read the sensors:
    for i in range(len(sensors)):
        print(sensors[i].getValue())
    
    linear.setPosition(0.08)
    
    position = navigation(initialPosition, sensors)
    for i in range(len(position) - 1):
        motors[i + 1].setPosition(position[i + 1])
    if position[0] == False:
        break
    
    if supervisor.getTime() > 2000:
        break

while supervisor.step(timestep) != -1:
    t = supervisor.getTime()
    # Read the sensors:
    for i in range(len(sensors)):
        print(sensors[i].getValue())

    # calling forward kinematics
    fk_results = armChain.forward_kinematics([0] * 8)
    # calling ikpy
    ik_results = armChain.inverse_kinematics(
        [0.5, 0.5, 0.5]
        )
    
    print(ik_results[:7])
    print(fk_results[:3, :3])    
    print(fk_results[:3, 3])
    
    # Robot arm Movement
    for i in range(6):
        motors[i].setPosition(ik_results[i+1])


# Enter here exit cleanup code.
