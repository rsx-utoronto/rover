#!/usr/bin/env python
import rospy, math,os
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
from ar_track_alvar_msgs.msg import *


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.markers)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.markers[0].pose.pose.position.x)
    
def listener():

    # In ROS, nodes are uniquely namflaged. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ar_data_read', anonymous=True)

    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
    rospy.loginfo("Starting to look for ar pos")    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
