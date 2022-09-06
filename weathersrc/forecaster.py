import numpy as np

def gen_components(file):
    # in the form speed, direction

    origional = np.genfromtxt(file, delimiter=',')
    u,v = np.zeros(len(origional)), np.zeros(len(origional))
    print(origional.shape)

    for i, c in enumerate(origional):
        as_rad = np.deg2rad(c[1])
        u[i] = c[0] * np.cos(as_rad)
        v[i] = c[0] * np.sin(as_rad)

    components = np.array(list(zip(u,v)))
    np.savetxt("weathersrc/weatherout/p1.csv", components)
    return components

gen_components("weathersrc/p1weatherin.csv")