#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial

def callback(data):
    rospy.loginfo(data.data)
    ser.write(data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('science_listener', anonymous=True)

    rospy.Subscriber('science', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    listener()
    ser.close()
