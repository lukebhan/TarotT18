import argparse
import math
import sys
import time

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Qt5Agg")
sys.path.append("src")
import matplotlib.pyplot as plt

from src.airframe import Airframe
from src.allocation import Allocator
from src.battery.tarotWrapper import TarotBattery
from src.controller.altitudeController import AltitudeController
from src.controller.attitudeController import AttitudeController
from src.controller.positionController import PositionController
from src.geoconversion import fromlatlon as geo
from src.motor.motor import Motor
from weathersrc.interpolator import interpolate


def run(x, y, z=45, filename="", output="out", ignore=False, weather=None):
    # Initialize Motors
    MotorParams = {
        "torqueConst": 0.0265,
        "equivResistance": 0.2700,
        "currentSat": 38,
        "staticFric": 0,
        "damping": 0,
        "J": 5.0e-5,
        "thrustCoeff": 0.065,
    }
    icMotor = 0
    sampleTime = 0.01
    motorArray = []
    for i in range(8):
        m = Motor(MotorParams, icMotor, sampleTime)
        motorArray.append(m)

    # Create three batteries
    bat = TarotBattery(sampleTime, f"{output}/{filename}-battery.txt")

    # Initialize Airframe
    TarotParams = {
        "g": 9.80,
        "m": 10.66,
        "l": 0.6350,
        "b": 9.8419e-05,
        "d": 1.8503e-06,
        "minAng": math.cos(math.pi / 8),
        "maxAng": math.cos(3 * math.pi / 8),
        "Ixx": 0.2506,
        "Iyy": 0.2506,
        "Izz": 0.4538,
        "maxSpeed": 670,
        "voltageSat": 0.0325,
    }
    tarot = Airframe(TarotParams, sampleTime)

    # Initialize Control Allocation
    allocator = Allocator(TarotParams)

    # Initial controllers
    AltitudeControllerParams = {"m": 10.66, "g": 9.8, "kdz": -1, "kpz": -0.5}
    altitudeController = AltitudeController(AltitudeControllerParams)
    PositionControllerParams = {
        "kpx": 0.1,
        "kdx": 0,
        "kpy": 0.1,
        "kdy": 0,
        "min_angle": -12 * math.pi / 180,
        "max_angle": 12 * math.pi / 180,
    }
    positionController = PositionController(PositionControllerParams)
    AttitudeControllerParams = {
        "kdphi": 1,
        "kpphi": 3,
        "kdpsi": 1,
        "kppsi": 3,
        "kdtheta": 1,
        "kptheta": 3,
    }
    attitudeController = AttitudeController(AttitudeControllerParams)

    xrefarr = x
    yrefarr = y

    zref = z
    # Psi ref = 0
    psiRef = 0

    # Simulate
    # time is in 1/100 of second
    x_true = []
    y_true = []
    errors = []
    xref = xrefarr[0]
    yref = yrefarr[0]
    np.savetxt(f"{output}/{filename}x-ref", xrefarr)
    np.savetxt(f"{output}/{filename}y-ref", yrefarr)
    state = tarot.getState()
    count = 0

    if weather is None:
        return
        # weather = np.zeros((len(x),3))

    # s_x = 0
    # s_y = 0
    # s_weather = weather[0]
    # e_weather = weather[1]
    wind = np.array(weather[0])

    f = 0

    # wind = [0,0,0]

    while f != len(xrefarr):
        # error = math.sqrt((xref-state[0])*(xref-state[0]) + (yref-state[1]) * (yref-state[1]))
        # errors.append(error)
        # if error < 1.5:
        #     # print(error)
        #     count += 1
        #     xref = xrefarr[count]
        #     yref = yrefarr[count]

        # wind = np.array(weather[count])
        # elif error > 15 and count > 10:
        #     break

        x_dists = xrefarr[f : min(len(xrefarr) - 1, f + 4)]
        y_dists = yrefarr[f : min(len(yrefarr) - 1, f + 4)]

        dists = np.zeros(len(x_dists))

        for n in range(len(x_dists)):
            dists[n] = np.sqrt(
                (state[0] - x_dists[n]) ** 2 + (state[1] - y_dists[n]) ** 2
            )

        try:
            f += np.argmin(dists)
        except Exception:
            print(dists)
            np.savetxt(f"{output}/{filename}x-flown", x_true)
            np.savetxt(f"{output}/{filename}y-flown", y_true)
            return errors
        xref = xrefarr[f]
        yref = yrefarr[f]
        error = math.sqrt(
            (xref - state[0]) * (xref - state[0])
            + (yref - state[1]) * (yref - state[1])
        )
        errors.append(error)
        if error <= 2:
            f += 1
        elif error > 12:
            print(error, f)
            np.savetxt(f"{output}/{filename}x-flown", x_true)
            np.savetxt(f"{output}/{filename}y-flown", y_true)
            return errors

        xref = xrefarr[f]
        yref = yrefarr[f]
        wind = np.array(weather[f])

        x_true.append(state[0])
        y_true.append(state[1])

        # Todo: Add wind interpolation here

        wind = [0, 0, 0]

        fz = altitudeController.output(state, zref)
        thetaRef, phiRef = positionController.output(state, [xref, yref])
        roll, pitch, yaw = attitudeController.output(state, [phiRef, thetaRef, psiRef])
        uDesired = [fz, roll, pitch, yaw]
        # print(uDesired)
        refVoltage = allocator.getRefVoltage(uDesired)
        # iterate over each motor
        rpm = np.zeros(8, dtype=np.float32)
        totalCurrent = 0
        for idx, motor in enumerate(motorArray):
            rpm[idx], current = motor.getAngularSpeed(refVoltage[idx])
            totalCurrent += current

        if not ignore:
            try:
                bat.step({"i": totalCurrent})
            except ValueError:
                break

        state = tarot.update(rpm, wind)
        # Add noise
        if i % 100 == 0:
            state[0] += np.random.normal(0, 0.2)
            state[1] += np.random.normal(0, 0.2)
    np.savetxt(f"{output}/{filename}x-flown", x_true)
    np.savetxt(f"{output}/{filename}y-flown", y_true)
    print(errors)
    return errors


def plot(output, filename, errors):
    x = np.loadtxt(f"{output}/{filename}x-flown")
    y = np.loadtxt(f"{output}/{filename}y-flown")
    xref = np.loadtxt(f"{output}/{filename}x-ref")
    yref = np.loadtxt(f"{output}/{filename}y-ref")

    plt.plot(xref, yref, label="Ref")
    plt.plot(x, y, label="Real")
    plt.legend()
    plt.savefig(f"{output}/{filename}trajectory.png")
    plt.show()

    bat1 = np.loadtxt(f"{output}/{filename}-battery.txt")

    plt.plot(bat1, label="Log Battery 1")
    plt.legend()
    plt.savefig(f"{output}/{filename}-battery.png")
    plt.show()

    plt.plot(errors, label="Path deviations")
    plt.legend()
    plt.savefig(f"{output}/{filename}-errors.png")
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pitcher files")
    parser.add_argument(
        "--file", "-f", dest="file", type=str, help="Path to the input file"
    )
    parser.add_argument(
        "--geo",
        "-g",
        dest="isgeo",
        type=str,
        help="Does the file contain latitude and longitude or is it in x, y",
        default="True",
    )
    parser.add_argument(
        "--out",
        "-t",
        dest="out",
        type=str,
        help="Path to the output folder",
        default="out",
    )
    parser.add_argument(
        "--id", "-i", dest="_id", type=str, help="Name of the run", default=""
    )
    parser.add_argument(
        "--plot",
        "-p",
        dest="plot",
        type=str,
        help="Should plot the outputs",
        default="True",
    )
    parser.add_argument(
        "--sample",
        "-s",
        dest="sample",
        type=int,
        help="Point sample interval (m) for coordinate generation",
        default=3,
    )

    args = parser.parse_args()

    print("Loading Coordinates...")
    coords = np.genfromtxt(args.file, delimiter=",")
    print("Coordinates loaded!")

    if args.isgeo == "True":
        print("Formatting Coordinates to x,y...")
        coords, weather = geo(coords, args.sample)
        print("Coordinates Formatted!")
    else:
        weather = None

    start = time.perf_counter()
    print("Running Simulation...")
    errors = run(
        coords[:, 1], coords[:, 0], filename=args._id, output=args.out, weather=weather
    )
    print("Done!")
    end = time.perf_counter()

    if args.plot == "True":
        print("Generating Plots...")
        plot(output=args.out, filename=args._id, errors=errors)

    # print(f'\nSimulation complete in: {((end-wall)/60):.2f}minutes')
    print("============ END ============")
