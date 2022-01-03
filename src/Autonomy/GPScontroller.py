#!/usr/bin/env python3
from PIDcontrol import AutonomousRover
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


def GPSListener():

	auto_rover = AutonomousRover(0, 0.05, 0.05)

	rospy.init_node('GPSListener', anonymous=True)

	rospy.Subscriber('/fix', NavSatFix, auto_rover.callbackGPS)
	rospy.spin()



if __name__ == '__main__':
	auto_rover = AutonomousRover(0, 0.05, 0.05)
	rospy.loginfo("defined rover")
	auto_rover.listener()
	rospy.spin()