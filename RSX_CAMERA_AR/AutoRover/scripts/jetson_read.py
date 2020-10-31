#!/usr/bin/env python
import rospy, math,os
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
from ar_track_alvar_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  
import cv2


def callback(image):
    # rospy.loginfo(rospy.get_caller_id() + "I say Hi")
    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(image, "bgr8")

    cv2.imshow('cv_img', orig)
    cv2.waitKey(2)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.markers[0].pose.pose.position.x)
    
def listener():

    # In ROS, nodes are uniquely namflaged. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Jetson_Image', anonymous=True)

    rospy.Subscriber("/camera/image_raw", Image, callback)
    rospy.loginfo("Please help, trying to see")    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
