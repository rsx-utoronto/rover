#!/usr/bin/env python

import rospy
from inertial_sense.msg import GPS
import serial
import math


def callback(data):
    maxAngle = 90
    latAnt = math.radians(0)
    lonAnt = math.radians(0)
    latRover = math.radians(-3)#math.radians(data.latitude)
    lonRover = math.radians(1)#math.radians(data.longitude)
    #rospy.loginfo("latRover: " + str(lat) + "lonRover: " + str(lon))
    X = math.cos(latRover)*math.sin(lonRover-lonAnt)
    Y = math.cos(latAnt)*math.sin(latRover)-math.sin(latAnt)*math.cos(latRover)*math.cos(lonRover-lonAnt)
    angle = math.degrees(math.atan(Y/X))
    if angle >0:
        angle = max(0,angle-maxAngle/2)
    else:
        angle = 90+maxAngle/2+angle
    print(angle)
    ser.write(str(int(angle)))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('comm_listener', anonymous=True)

    rospy.Subscriber('gps', GPS, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    #listener()
    callback(1)
    ser.close()
