import argparse
import math
import sys
import time

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use('Qt5Agg')
sys.path.append('src')
import matplotlib.pyplot as plt

from airframe import Airframe
from allocation import Allocator
from battery.tarotWrapper import TarotBattery
from controller.altitudeController import AltitudeController
from controller.attitudeController import AttitudeController
from controller.positionController import PositionController
from geoconversion import fromlatlon as geo
from motor.motor import Motor


def run(x,y,z=3, filename = "", output = "out"):
    # Initialize Motors
    MotorParams = {"torqueConst": 0.0265, "equivResistance": 0.2700, "currentSat": 38, "staticFric": 0, "damping": 0, "J": 5.0e-5, "thrustCoeff": 0.065}
    icMotor = 0
    sampleTime = 0.01
    motorArray = []
    for i in range(8):
        m = Motor(MotorParams, icMotor, sampleTime)
        motorArray.append(m)

    # Create three batteries
    bat = TarotBattery(sampleTime, f'{output}/{filename}-battery.txt')

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

    xrefarr = x
    yrefarr = y

    zref = 3
    # Psi ref = 0
    psiRef = 0

    # Simulate 
    # time is in 1/100 of second
    arr1 = []
    arr2 = []
    xref = xrefarr[0]
    yref = yrefarr[0]
    np.savetxt(f'{output}/{filename}x-ref', xrefarr)
    np.savetxt(f'{output}/{filename}y-ref', yrefarr)
    state = tarot.getState()
    count = 0
    while count != len(xrefarr)-1:
        error = math.sqrt((xref-state[0])*(xref-state[0]) + (yref-state[1]) * (yref-state[1]) + (zref-state[2]) * (zref-state[2]))
        if error < 0.5:
            count += 1
            xref = xrefarr[count]
            yref = yrefarr[count]
        arr1.append(state[0])
        arr2.append(state[1])
        fz = altitudeController.output(state, zref)
        thetaRef, phiRef = positionController.output(state, [xref, yref])
        roll, pitch, yaw = attitudeController.output(state, [phiRef, thetaRef, psiRef]) 
        uDesired = [fz, roll, pitch, yaw]
        refVoltage = allocator.getRefVoltage(uDesired)
        # iterate over each motor
        rpm = np.zeros(8, dtype=np.float32)
        totalCurrent = 0
        for idx, motor in enumerate(motorArray):
            rpm[idx], current  = motor.getAngularSpeed(refVoltage[idx])
            totalCurrent +=  current
        bat.step({"i": totalCurrent})
        state = tarot.update(rpm)
        # Add noise 
        if i  % 100 == 0:
            state[0] += np.random.normal(0, 0.1)
            state[1] += np.random.normal(0, 0.1)
    np.savetxt(f'{output}/{filename}x-flown', arr1)
    np.savetxt(f'{output}/{filename}y-flown', arr2)

def plot(output, filename):
    x = np.loadtxt(f'{output}/{filename}x-flown')
    y = np.loadtxt(f'{output}/{filename}y-flown')
    xref = np.loadtxt(f'{output}/{filename}x-ref')
    yref = np.loadtxt(f'{output}/{filename}y-ref')

    plt.plot(x, y, label="Real")
    plt.plot(xref, yref, label="Ref")
    plt.legend()
    plt.savefig(f'{output}/{filename}trajectory.png')
    plt.show()

    bat1 = np.loadtxt(f'{output}/{filename}-battery.txt')

    plt.plot(np.log(bat1), label="Log Battery 1")
    plt.legend()
    plt.savefig(f'{output}/{filename}-battery.png')
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Pitcher files')
    parser.add_argument('--file','-f' , dest='file', type=str, help='Path to the input file')
    parser.add_argument('--geo','-g' , dest='isgeo', type=str, 
    help='Does the file contain latitude and longitude or is it in x, y', default='True')
    parser.add_argument('--out','-t' , dest='out', type=str, help='Path to the output folder', default = "out")
    parser.add_argument('--id','-i' , dest='_id', type=str, help='Name of the run', default = "")
    parser.add_argument('--plot','-p' , dest='plot', type=str, help='Should plot the outputs', default = "True")
    parser.add_argument('--sample','-s' , dest='sample', type=int, help='Point sample interval (m) for coordinate generation', default = 3)

    args = parser.parse_args()

    print("Loading Coordinates...")
    coords = np.genfromtxt(args.file,delimiter=',')
    print("Coordinates loaded!")
    
    if args.isgeo == "True":
        print("Formatting Coordinates to x,y...")
        coords = geo(coords, args.sample)
        print("Coordinates Formatted!")
    
    start = time.perf_counter()
    print("Running Simulation...")
    run(coords[:,1], coords[:,0], filename=args._id, output=args.out)
    print("Done!")
    end = time.perf_counter()

    if args.plot == "True":
        print("Generating Plots...")
        plot(output=args.out,filename=args._id)
    
    print(f'\nSimulation complete in: {((end-wall)/60):.2f}minutes')
    print("============ END ============")
