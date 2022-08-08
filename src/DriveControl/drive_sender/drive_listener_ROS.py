#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial
import argparse



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' Message: %s', data.data)
    ser.write(data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('drive_listener', anonymous=True)
    rospy.Subscriber('drive', String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    arduino_port = rospy.get_param('arduino_port') #should return 1
    baud_rate = rospy.get_param("baud_rate")
    ser = serial.Serial(arduino_port, baud_rate)
    listener()
    ser.close()