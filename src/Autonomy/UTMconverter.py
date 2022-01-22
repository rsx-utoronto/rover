#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from inertial_sense.msg import GPS
from sensor_msgs.msg import MagneticField
import threading
import math
import time
import geodesy
class UTMConverter:

    def __init__(self, tick, errorx, errory, server):
        self.easting = 0
        self.northing = 0 
        self.zone = 'unknown'
        self.altitude = 0
        self.band = 'unknown'

        self.pub_UTM = rospy.Publisher("utm", UTMPoint, queue_size=10)

    def callbackGPS(data):
        # If print doesn't work use loginfo
        # rospy.loginfo(rospy.get_name()+ "Coordinates are x=%f y=%f z=%f", data.pose.pose.position.x,
        #                                                                  data.pose.pose.position.y,
        #                                                                  data.pose.pose.position.z)
        geo_pt = GeoPoint()
        geo_pt.latitude = data.latitude
        geo_pt.longitude = data.longitude
        geo_pt.altitude = data.altitude
        utm_pt = UTMPoint(geo_pt)
        self.easting = utm_pt.easting
        self.northing = utm_pt.northing
        self.altitude = utm_pt.altitude
        self.zone = utm_pt.zone
        self.band = utm_pt.band

        self.pub.pub_UTM(utm_pt)

    def listener():
        rospy.init_node('gps_listener', anonymous=True)
        rospy.Subscriber('/gps', GPS, callbackGPS)
        rospy.spin()   #see if I can change frequency to slower than inertial sense provision

if __name__ == '__main__':
    UTM_converter = UTMConverter()
    while not rospy.is_shutdown():
        UTM_converter.listener() 