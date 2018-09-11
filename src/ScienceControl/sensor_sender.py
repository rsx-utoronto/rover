#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial

def sender():

    pub = rospy.Publisher('science', String, queue_size=10)
    rospy.init_node('sensor_sender', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        line = ser.readline()
        if line != None:
            rospy.loginfo(line)
            pub.publish(line)
            rate.sleep()


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
    ser.close()
