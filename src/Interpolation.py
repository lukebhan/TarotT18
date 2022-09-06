from cmath import cos, cosh, pi
import json
from xml.dom.pulldom import PROCESSING_INSTRUCTION
from geopy.distance import geodesic
import math
import numpy as np
import datetime

def Interpolation(position, waypoint):
    """A function that takes a dictionary of a route and inserts intermediate points.

    position is a dictionary consisting of the lat / lon / simulated time (in unix) / etc. of the model
    waypoint is the lat, lon, alt, etc. of the next waypoint
    """
    with open('DATA/WAYPOINTS/waypoints.json') as json_file:
        waypoint_file = json.load(json_file)
    waypoints = waypoint_file["waypoints"]
    with open('DATA/ROUTES/rte1.json') as json_file:
        route_file = json.load(json_file)
    route = route_file[next(iter(route_file))]
    with open('DATA/WEATHER/rte1.weather.json') as json_file:
        weather = json.load(json_file)
    current_lat = position['lat']
    current_lon = position['lon']
    p = current_lat, current_lon
    next_waypoint = waypoint['lat'], waypoint['lon']
    wpt1 = waypoints[0]["lat"], route["timestamps"][0]
    wpt2 = waypoints[1]["lat"], route["timestamps"][1]
    time_current = position['time']
    closest_time = datetime.datetime.timestamp(datetime.datetime.strptime(route['timestamps'][0], '%Y-%m-%dT%H:%M:%S%z'))

    index = 0
    for i in range(len(route['timestamps'])):
        stamp = datetime.datetime.strptime(route['timestamps'][i], '%Y-%m-%dT%H:%M:%S%z')

        time = datetime.datetime.timestamp(stamp)
        if (waypoints[i]['lat'], waypoints[i]['lon']) == next_waypoint and abs(time_current - time) < abs(time_current - closest_time):
            index = i
            closest_time = time
            wpt1 = waypoints[i - 1]["lat"], route["timestamps"][i - 1]
            wpt2 = waypoints[i]["lat"], route["timestamps"][i]
    for key in weather:
        if weather[key]["lat"] == wpt1[0] and weather[key]["timestamp"] == wpt1[1]:
            weather1 = weather[key]
            start = int(key)
        if weather[key]["lat"] == wpt2[0] and weather[key]["timestamp"] == wpt2[1]:
            weather2 = weather[key]
            end = int(key)
    weatherpt1 = None
    weatherpt2 = None
    i = start
    for _ in range(end - i):
        weatherpt1 = weather1["lat"], weather1["lon"]
        weatherpt2 = weather2["lat"], weather2["lon"]
        point = weather[str(i)]["lat"], weather[str(i)]["lon"]
        if geodesic(weatherpt1, p) > geodesic(point, p):
            weather1 = weather[str(i)]
            pointC = weather[str(i - 1)]["lat"], weather[str(i - 1)]["lon"]
            pointD = weather[str(i + 1)]["lat"], weather[str(i + 1)]["lon"]
            if geodesic(pointC, p) > geodesic(pointD, p):
                weather2 = weather[str(i + 1)]
            else:
                weather2 = weather[str(i - 1)]
        i += 1
    weatherpt1 = weather1["lat"], weather1["lon"]
    weatherpt2 = weather2["lat"], weather2["lon"]
    pd = geodesic(weatherpt1, p).m
    wd = geodesic(weatherpt1, weatherpt2).m
    alpha = pd / wd
    angle1 = 90 - weather1["wind_dir"]
    angle1 = angle1 % 360
    angle2 = 90 - weather2["wind_dir"]
    angle2 = angle2 % 360
    u1 = weather1["wind_speed"] * math.cos(math.radians(angle1))
    v1 = weather1["wind_speed"] * math.sin(math.radians(angle1))
    u2 = weather2["wind_speed"] * math.cos(math.radians(angle2))
    v2 = weather2["wind_speed"] * math.sin(math.radians(angle2))
    mu = (1 - math.cos(alpha * pi)) / 2
    u = u1 * (1 - mu) + u2 * mu
    v = v1 * (1 - mu) + v2 * mu
    return u, v