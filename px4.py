#!/usr/bin/env python
import rospy
import json
import random
from geopy.distance import distance as gps_distance
from mavros_msgs.srv import (CommandBool, ParamGet, SetMode,
                             WaypointClear, WaypointPush)
from mavros_msgs.msg import (Altitude, ExtendedState, Waypoint,
                             HomePosition, State, WaypointList)
from sensor_msgs.msg import NavSatFix

mission_waypoints = []
with open('/home/mradovan/catkin_ws/src/mavros/mavros/scripts/sample_mission_1.json', 'r') as f:
    mission_waypoints = json.load(f)
    mission_waypoints = [Waypoint(*([0, 16, False, True] + w)) for (idx, w) in enumerate(mission_waypoints)]

time_step = .01
HOVER_LAND_SECONDS = 10
GPS_TOLERANCE = 0.0000001
DISTANCE_TOLERANCE = 2#1.25

# USAGE:
# self.get_param_srv('MAV_TYPE')
# self.set_arming_srv(True)
# self.set_mode_srv(0, mode) # 0 is custom mode
# http://wiki.ros.org/mavros/CustomModes
# self.wp_clear_srv()
# self.wp_push_srv(start_index=n, waypoints=[Waypoint()...])

class PX4:
    def __init__(self):
        rospy.init_node('PX4', anonymous=True)
        self.initial_pos = None
        self.left_home = False
        self.back_home_time = None
        # self.can_land = False
        self.mission_waypoints = mission_waypoints

        # mavros stuff
        self.gps_data_received = rospy.Subscriber('/mavros/global_position/global/', NavSatFix, self.receive_gps)
        self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.wp_clear_srv = rospy.ServiceProxy('mavros/mission/clear',
                                               WaypointClear)
        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push',
                                              WaypointPush)

        # custom in/out stuff
        self.waypoints_received = rospy.Subscriber('waypoint_data', WaypointList, self.receive_waypoints)

    def near_base(self, current, base):
        distance = gps_distance((current.latitude, current.longitude), (base.latitude, base.longitude))
        meter_distance = distance.km * 1000.0
        return meter_distance <= DISTANCE_TOLERANCE

    # once it leaves the home position (if the lat and long have left the radius of home position)
    # if it comes back in a certain range (lat+lng wise) and holds for N seconds, then land
    def receive_gps(self, data):
        if not self.initial_pos:
            self.initial_pos = data 
        elif not self.near_base(data, self.initial_pos):
            self.left_home = True
            self.back_home_time = None
        elif self.left_home:
            if self.back_home_time:
                time_passed = data.header.stamp.secs - self.back_home_time
                if time_passed > HOVER_LAND_SECONDS:
                    self.set_mode_srv(0, 'AUTO.LAND')
            else:
                self.back_home_time = data.header.stamp.secs

    def receive_waypoints(self, waypoints):
        self.wp_push_srv(start_index=0, waypoints=waypoints)

    def run(self):
        self.set_arming_srv(True)
        self.set_mode_srv(0, 'AUTO.TAKEOFF')
        self.wp_clear_srv()
        self.wp_push_srv(start_index=0, waypoints=mission_waypoints)
        self.set_mode_srv(0, 'AUTO.MISSION')
        
        # seeing what happens when we add a waypoint.
        # rospy.Rate(1.0/7.0).sleep()
        # new_waypoints = mission_waypoints + [Waypoint(0, 16, False, True, 0, 0, 0, None, 47.39773941040039 + random.random() *.001, 8.5455904006958 + random.random() *.001, 510)]
        # print(new_waypoints)
        # self.wp_push_srv(start_index=0, waypoints=new_waypoints)

        rate = rospy.Rate(1.0/time_step) # 100 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    mavros_proxy = PX4()
    mavros_proxy.run()