import numpy as np

def interpolate(ac_x, ac_y, s_x, s_y, e_x, e_y, s_weather, e_weather):
    # Distance based interpolation
    ac_dist = np.sqrt(((ac_x-s_x)**2) + ((ac_y-s_y)**2))
    trav_dist = np.sqrt(((e_x-s_x)**2) + ((e_y-s_y)**2))

    u = s_weather[0]+ac_dist*((e_weather[0]-s_weather[0])/trav_dist)
    v = s_weather[1]+ac_dist*((e_weather[1]-s_weather[1])/trav_dist)
    return np.array([u,v,0.0])