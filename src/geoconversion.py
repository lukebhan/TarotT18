import numpy as np
from geographiclib.geodesic import Geodesic
import math
import matplotlib.pyplot as plt
import pandas as pd
from weathersrc.interpolator import interpolate

path = "examples/paths/"
name = "TEST"

def fromlatlon(route, sample=3):
    unfiltered = np.genfromtxt("weathersrc/weatherout/p1.csv", delimiter=' ')
    weather = [np.array([unfiltered[0,0], unfiltered[0,1], 0])]
    coords = [[0,0]]
    print(len(route), len(unfiltered))
    for i in range(1,len(route)):
        e = unfiltered[i]
        s = unfiltered[i-1]

        t = Geodesic.WGS84.Inverse(route[i-1,0],route[i-1,1], route[i,0], route[i,1])

        dist = t['s12']
        heading = t['azi1']

        ox,oy = coords[-1][0],coords[-1][1]

        ex = ox+((dist*2)*math.cos(math.radians(heading)))
        ey = oy+((dist*2)*math.sin(math.radians(heading)))

        print(ex, ey)

        for j in range(1,(int(dist/sample)+1)):
            v = j

            if v > dist:
                v=dist
            x = ox+((v*2)*math.cos(math.radians(heading)))
            y = oy+((v*2)*math.sin(math.radians(heading)))
            coords.append([x,y])

            weather.append(interpolate(x,y,ox,oy,ex,ey,s,e))



    coords = np.array(coords)
    x = coords[:,0]
    y = coords[:,1]

    print(weather)
    return coords, weather

    # df = pd.DataFrame(data=x)
    # df.index = np.arange(1,len(df)+1)
    # df.to_csv(f'{path}{name}x.csv', mode='w', index=True, header=False)
    # df = pd.DataFrame(data=y)
    # df.index = np.arange(1,len(df)+1)
    # df.to_csv(f'{path}{name}y.csv', mode='w', index=True, header=False)

    # plt.plot(coords[:,0],coords[:,1])
    # plt.show()
