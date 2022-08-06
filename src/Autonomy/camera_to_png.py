#! /usr/bin/python

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

import argparse

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("ZED camera image received!")
    try:
        # Convert your ROS Image message to OpenCV2
        # check ROS wiki for diff message types, bgr8 is color image with blue-green-red color order
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8") 
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a png
        cv2.imwrite("camera_image.png", cv2_img)

def main():
    rospy.init_node('cam_to_png')
    # Define ZED camera image topic
    image_topic = "/zed/zed_node/right_raw/image_raw_color"

    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--start_time", help = "-s: start time")
    parser.add_argument("-e", "--end_time", help = "-e: end time")

    args = parser.parse_args()
    print("Start time: ", args.start_time)
    print("End time: ", args.end_time)

    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()