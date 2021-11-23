import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

x = np.loadtxt("x")
y = np.loadtxt("y")
xref = np.loadtxt("xref")
yref = np.loadtxt("yref")

plt.plot(x, y, label="Real")
plt.plot(xref, yref, label="Ref")
plt.legend()
plt.savefig("trajectory.png")
plt.show()

bat1 = np.loadtxt("battery.txt")

plt.plot(bat1, label="Battery 1")
plt.legend()
plt.savefig("battery.png")
plt.show()

