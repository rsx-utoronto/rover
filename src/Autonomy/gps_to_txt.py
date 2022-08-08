#! /usr/bin/python

import rospy
from sensor_msgs.msg import NavSatFix

import argparse
import os

def gps_callback(gps_msg):
    # save GPS message to text file
    print("GPS message received")
    os.system("rostopic echo  > gps.txt")


def main():
    rospy.init_node("gps_to_txt")

    # gps rostopic name
    gps_topic = "" 
    rospy.Subscriber(gps_topic, NavSatFix, gps_callback)
    rospy.spin()

if __name__ == '__main__':
    main()





