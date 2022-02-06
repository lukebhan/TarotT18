import numpy as np
from geographiclib.geodesic import Geodesic
import math
import matplotlib.pyplot as plt
import pandas as pd

path = "examples/paths/"
name = "TEST"

def fromlatlon(route, sample=3):
    # In the form [[x1,x2,...],[y1,y2,...]]
    coords = [[0,0]]

    for i in range(1,len(route)):
        t = Geodesic.WGS84.Inverse(route[i-1,0],route[i-1,1], route[i,0], route[i,1])

        dist = t['s12']
        heading = t['azi1']

        ox,oy = coords[-1][0],coords[-1][1]

        for j in range(1,(int(dist/sample)+1)):
            v = j

            if v > dist:
                v=dist
            x = ox+((j*2)*math.cos(math.radians(heading)))
            y = oy+((j*2)*math.sin(math.radians(heading)))
            coords.append([x,y])

        

    coords = np.array(coords)
    x = coords[:,0]
    y = coords[:,1]

    return (coords)

    # df = pd.DataFrame(data=x)
    # df.index = np.arange(1,len(df)+1)
    # df.to_csv(f'{path}{name}x.csv', mode='w', index=True, header=False)
    # df = pd.DataFrame(data=y)
    # df.index = np.arange(1,len(df)+1)
    # df.to_csv(f'{path}{name}y.csv', mode='w', index=True, header=False)

    # plt.plot(coords[:,0],coords[:,1])
    # plt.show()


fromlatlon(np.array([[53.61031,-2.10288],[53.6098,-2.10365],[53.60966,-2.10428],
[53.60953,-2.10440],[53.60853,-2.10389],[53.60862,-2.10321],[53.60826,-2.10239],
[53.60900,-2.10116],[53.60931,-2.10022],[53.60952,-2.10042],[53.60906,-2.10216],
[53.60903,-2.10362],[53.60927,-2.10373],[53.60947,-2.10268],[53.60965,-2.10215],
[53.60984,-2.10216],[53.61031,-2.10288]]))
