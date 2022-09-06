from typing import List
import numpy as np


class Weather:
    def __init__(self, Cd: List[float], A: List[float]):
        self.cd = np.array(Cd)
        self.A = np.array(A)

        self.kB = 1.38064852e-23
        self.mD = 4.81e-26

    def get_drag(self, v: List[float], wv: List[float], rho: float = 1.225):
        v, wv = np.array(v), np.array(wv)

        a = self.cd*self.A

        relv = (v - wv) ** 2

        return 0.5*(rho*relv*a)

    def get_rhoT(self, T, P):
        "Get rho based on Temperature in C and pressure in kPa"

        RSpecific = self.kB/self.mD

        T = T + 273.15
        P = P*1000
        top = P*self.mD
        bottom = self.kB*T

        return top/bottom


w = Weather([0.606, 0.702, 1.192], [0.062214, 0.0698, 0.167784])
rho = w.get_rho(20, 101.325)
print(rho)
print(w.get_drag([13, 0, 0], [-8, -6, 0], rho))
