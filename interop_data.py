#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from mavros_msgs.msg import Waypoint, WaypointList
from msgs import LatLng, Obstacle, LatLngList, ObstacleList

import pathplanning

import requests

API_URI = '<CHANGEME>'
MISSION_ID = 1
USERNAME = '<CHANGEME>'
PASSWORD = '<CHANGEME>'

with open('config.json', 'r') as f:
    config = json.load(f)

# Waypoint(
#     reference frame (0 for latlng),
#     command (16 as waypoint),
#     is current waypoint,
#     continue to next automatically,
#     idk, idk, idk, idk,
#     lat, lng, alt
# )

def latlngs_to_wps(points, altitude):
    wps = []
    for point in points:
        wp = Waypoint(
            0, 16, False, True,
            0, 0, 0, None,
            point['latitude'], point['longitude'], altitude
        )
        wps.append(wp)
    return wps

def latlngs_to_array(points):
    rpoints = []
    for p in points:
        rpoints.append([points['latitude'], points['longitude']])
    rpoints = np.array(rpoints)
    return rpoints.T

def obstacles_to_array(points):
    rpoints = []
    for p in points:
        rpoints.append([points['latitude'], points['longitude'], points['radius']])
    rpoints = np.array(rpoints)
    return rpoints.T

time_step = 1.0 # every second
class PullInteropData:
    def __init__(self):
        rospy.init_node('PullInteropData', anonymous=True)
        r = requests.post('{}/api/login'.format(API_URI), data={
            'username': USERNAME,
            'password': PASSWORD
        })
        self.cookies = r.cookies
        self.send_waypoints = rospy.Publisher('waypoint_data', WaypointList)
        self.send_obstacle_data = rospy.Publisher('obstacle_data', ObstacleList)
        self.send_boundary_data = rospy.Publisher('boundary_data', LatLngList)

    def run(self):
        response = requests.get('{}/api/missions/{}'.format(API_URI, MISSION_ID))
        data = response.json()
        boundary = latlngs_to_array(data['flyZones'][0]['boundaryPoints'])
        begin = data['waypoints'][0]

        response = requests.get('{}/api/obstacles'.format(API_URI))
        data = response.json()
        obstacles = latlngs_to_array(data['stationaryObstacles'])

        environment = Environment(boundary, obstacles, config.get('granularity', 100))
        path = pathplanning.build_path(begin, end)
        waypoint_path = latlng_arr_to_wps(path)
        self.send_waypoints(waypoint_path)

if __name__ == '__main__':
    node = PullInteropData()
    node.run()