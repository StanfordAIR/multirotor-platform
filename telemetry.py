#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

import requests

API_URI = '<CHANGEME>'
USERNAME = '<CHANGEME>'
PASSWORD = '<CHANGEME>'

time_step = 1.0 # every second
class SendTelemInfo:
    def __init__(self):
        rospy.init_node('Telemetry', anonymous=True)
        r = requests.post('{}/api/login'.format(API_URI), data={
            'username': USERNAME,
            'password': PASSWORD
        })
        self.cookies = r.cookies
        self.data = {'telem': None, 'heading': None}
        self.gps = rospy.Subscriber('/mavros/global_position/global/', NavSatFix, self.set_telem)
        self.heading = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.set_heading)

    def set_telem(self, data):
        self.data['telem'] = data
        if self.data.get('heading'):
            self.send_data()

    def set_heading(self, data):
        self.data['heading'] = data
        if self.data.get('telem'):
            self.send_data()

    def send_data(self):
        # needs to be x-www-form-urlencoded
        telem = self.data['telem']
        heading = self.data['heading']
        r = requests.post(
            '{}/api/telemetry'.format(API_URI),
            cookies=self.cookies,
            data={
                'latitude': telem.latitude,
                'longitude': telem.longitude,
                'altitude_msl': telem.altitude,
                'uas_heading': heading
            })
        # print(self.data['telem'])
        # print(self.data['heading'])
        # return

    def run(self):
        rate = rospy.Rate(1.0/time_step) # 100 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    telem = SendTelemInfo()
    telem.run()