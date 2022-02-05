#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from inertial_sense_ros.msg import GPS
from sensor_msgs.msg import MagneticField
import geodesy
from geographic_msgs.msg import GeoPoint
import threading
import math
import time

class UTMConverter:

    def __init__(self):
        self.easting = 0
        self.northing = 0 
        self.zone = 'unknown'
        self.altitude = 0
        self.band = 'unknown'

        self.pub_gps_utm = rospy.Publisher("gps_utm", UTMPoint, queue_size=10)
        self.pub_target_utm = rospy.Publisher("target_utm", UTMPoint, queue_size=10)

    def callbackGPS(data, target):
    
        geo_pt = GeoPoint()
        geo_pt.latitude = data.latitude
        geo_pt.longitude = data.longitude
        geo_pt.altitude = data.altitude
        utm_pt = geodesy.utm.UTMPoint(geo_pt)
        self.easting = utm_pt.easting
        self.northing = utm_pt.northing
        self.altitude = utm_pt.altitude
        self.zone = utm_pt.zone
        self.band = utm_pt.band

        if target:
            self.pub.pub_target_utm(utm_pt)
        else:
            self.pub.pub_gps_utm(utm_pt)

    def listener():
        rospy.init_node('gps_listener', anonymous=True)
        rospy.Subscriber('/gps', GPS, callbackGPS, (False))
        rospy.Subscriber('/target', GPS, callbackGPS, (True))
        rospy.spin()   #see if I can change frequency to slower than inertial sense provision

if __name__ == '__main__':
    UTM_converter = UTMConverter()
    while not rospy.is_shutdown():
        UTM_converter.listener() 