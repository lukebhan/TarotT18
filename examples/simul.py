import numpy as np
import math
import pandas as pd
import sys
sys.path.append('../src')
from motor.motor import Motor
from airframe import Airframe
from allocation import Allocator
from controller.altitudeController import AltitudeController
from controller.positionController import PositionController
from controller.attitudeController import AttitudeController
from battery.tarotWrapper import TarotBattery

# Initialize Motors
MotorParams = {"torqueConst": 0.0265, "equivResistance": 0.2700, "currentSat": 38, "staticFric": 0, "damping": 0, "J": 5.0e-5, "thrustCoeff": 0.065}
icMotor = 0
sampleTime = 0.01
motorArray = []
for i in range(8):
    m = Motor(MotorParams, icMotor, sampleTime)
    motorArray.append(m)

# Create three batteries
bat = TarotBattery(sampleTime, "battery.txt")

# Initialize Airframe
TarotParams = {"g": 9.80, "m": 10.66, "l": 0.6350, "b": 9.8419e-05, "d": 1.8503e-06, "minAng": math.cos(math.pi/8), "maxAng": math.cos(3*math.pi/8), "Ixx": 0.2506, "Iyy": 0.2506, "Izz": 0.4538, "maxSpeed": 670, "voltageSat": 0.0325}
tarot = Airframe(TarotParams, sampleTime)

# Initialize Control Allocation
allocator = Allocator(TarotParams)

# Initial controllers
AltitudeControllerParams = {'m': 10.66, 'g':9.8, 'kdz': -1, 'kpz': -0.5}
altitudeController = AltitudeController(AltitudeControllerParams)
PositionControllerParams = { 'kpx': 0.1, 'kdx': 0, 'kpy': 0.1, 'kdy': 0, 'min_angle': -12*math.pi/180, 'max_angle': 12*math.pi/180 }
positionController = PositionController(PositionControllerParams)
AttitudeControllerParams = {"kdphi": 1, "kpphi": 3, "kdpsi": 1, "kppsi": 3, "kdtheta": 1, "kptheta": 3}
attitudeController = AttitudeController(AttitudeControllerParams)

# Build Trajectories. Currently have 3 
traj = "zig"
if traj == '8':
    xrefarr = pd.read_csv("paths/xref8traj.csv", header=None).iloc[:, 1]
    yrefarr = pd.read_csv("paths/yref8traj.csv", header=None).iloc[:, 1]
elif traj == 'e':
    xrefarr = pd.read_csv("paths/xrefEtraj.csv", header=None).iloc[:, 1]
    yrefarr = pd.read_csv("paths/yrefEtraj.csv", header=None).iloc[:, 1]
elif traj == "zig":
    yrefarr = pd.read_csv("paths/yrefZigtraj.csv", header=None).iloc[:, 1]
    xrefarr = pd.read_csv("paths/xrefZigtraj.csv", header=None).iloc[:, 1]

zref = 3
# Psi ref = 0
psiRef = 0

# Simulate 
# time is in 1/100 of second
arr1 = []
arr2 = []
# Set the initial starting point (x & y) equal to the points from the ref arrays
xref = xrefarr[0]
yref = yrefarr[0]
# Store the reference trajectories
np.savetxt(f'xref', xrefarr)
np.savetxt(f'yref', yrefarr)
# Get initial state of the drone
state = tarot.getState()
count = 0

# Loop while there is still a waypoint in xref
while count != len(xrefarr)-1:
    # Get error/offset of current pos to ref pos
    error = math.sqrt((xref-state[0])*(xref-state[0]) + (yref-state[1]) * (yref-state[1]) + (zref-state[2]) * (zref-state[2]))
    # lower bound on error?
    if error < 0.5:
        count += 1
        xref = xrefarr[count]
        yref = yrefarr[count]
    # Add T18 position as the state
    arr1.append(state[0])
    arr2.append(state[1])
    # Get the ref voltages using the alt, att and pos controllers
    fz = altitudeController.output(state, zref)
    thetaRef, phiRef = positionController.output(state, [xref, yref])
    roll, pitch, yaw = attitudeController.output(state, [phiRef, thetaRef, psiRef]) 
    uDesired = [fz, roll, pitch, yaw]
    refVoltage = allocator.getRefVoltage(uDesired)
    # iterate over each motor (8 on the T18) and update total current
    rpm = np.zeros(8, dtype=np.float32)
    totalCurrent = 0
    for idx, motor in enumerate(motorArray):
        rpm[idx], current  = motor.getAngularSpeed(refVoltage[idx])
        totalCurrent +=  current
    # step the battery with the current
    bat.step({"i": totalCurrent})
    # Update the state using the rpm array
    state = tarot.update(rpm)
    # Add noise 
    if i  % 100 == 0:
        state[0] += np.random.normal(0, 0.1)
        state[1] += np.random.normal(0, 0.1)
# Save the x and y arrays
np.savetxt('x', arr1)
np.savetxt('y', arr2)
