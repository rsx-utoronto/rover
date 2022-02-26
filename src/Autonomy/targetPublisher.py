#!/usr/bin/python3
from time import sleep
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix
from geodesy.utm import UTMPoint, fromLatLong
from geographic_msgs.msg import GeoPoint
import geodesy
import pyproj


class Target:
    def __init__(self):
        self.init_target = True
        self.target = NavSatFix()
        self.utm_pt = UTMPoint()

        self.pub_target = rospy.Publisher("/target_gps", NavSatFix, queue_size=10)

    def targetListener(self):
        rospy.Subscriber('/fix', NavSatFix, self.targetCallback)

    def targetCallback(self, data):
        if self.init_target: 
            self.utm_pt = fromLatLong(data.latitude, data.longitude, data.altitude)
            print("INITIAL UTM POSITION: \n", self.utm_pt)

            self.utm_pt.easting += 200
            self.utm_pt.northing += 200
            print("UTM GOAL: \n", self.utm_pt)
            geo_pt = self.utm_pt.toMsg()
            self.target = NavSatFix()

            self.target.latitude = geo_pt.latitude
            self.target.longitude = geo_pt.longitude
            self.target.altitude = data.altitude
            self.init_target = False
        self.pub_target.publish(self.target)
        
    
if __name__ == '__main__':
    rospy.init_node('target_publisher', anonymous=True)
    target = Target()
    while not rospy.is_shutdown():
        target.targetListener()  