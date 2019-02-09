#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
from datetime import datetime
import json
import math
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
from distance_calc_cv import find_distance_given_diameter
from angle_calc_cv import angle_cal
from ball_trackingrw import *
from autonomousrover import *

def move_towards_the_ball(server, destination_markers, xError, yError, headError, center_of_frame, time_delay_between_markers, DIST_FROM_TENNISBALL):

    # Initialize publisher
    
    pub = rospy.Publisher('drive', Twist, queue_size = 10)    #create custom msg for stable publishing
    rospy.init_node('speed_talker', anonymous=true)
    rate = rospy.Rate(10) # Frequency: 10Hz

    # Construct an AutonomousRover Object
    rover = AutonomousRover(datetime.now(), xError, yError, server)
    marker_number = 0
    
    for marker in destination_markers:

        marker_number += 1
        print("marker: " + str(marker_number))

        #GPS Maneuver.
        arrived_at_destination = False
        while (not arrived_at_destination):
            arrived_at_destination = rover.move_towards_gps_Location(marker)

        print("Arrived at the GPS Destination")
        # Start searching for the ball.
        # construct the argument parse and parse the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-v", "--video",
                        help="path to the (optional) video file")
        ap.add_argument("-b", "--buffer", type=int, default=64,
                        help="max buffer size")
        args = vars(ap.parse_args())

        # if a video path was not supplied, grab the reference to the webcam
        if not args.get("video", False):
            camera = cv2.VideoCapture('http://192.168.0.101:15213/videostream.cgi?loginuse=admin&loginpas=wavesharespotpear')
        # otherwise, grab a reference to the video file
        else:
            camera = cv2.VideoCapture(args["video"])
        # ------- camera matrix comes from calibration algorithm from distance_calc_cv.py (find_intrinsic_matrix function)
        # or found in camera's doc ------------------
        # kmtx = np.load('C:/Users/rache/PycharmProjects/rsx/k_matrix.npy') #reading in the camera matrix from sally's iphone
        kmtx = np.load('v_matrix.npy')  # reading in the camera matrix, from the vesky camera
        calib_list = kmtx[0, 0], kmtx[1, 1], kmtx[0, 2], kmtx[1, 2]

        dis = 10000
        while(dis > DIST_FROM_TENNISBALL):
            print("Closing on the distance")
            while(True):

                #dis is the distance of the rover from the tennis ball
                #ang is the angle location of the tennis ball relative to the rover's heading.
                #From the frame perspective, angle starts from the right edge of the frame.
                (dis, ang, ball_found) = BallTrack(camera, args,calib_list)
                #Angle is from 0 to 53 degrees
                if (not ball_found):
                    print("looking for the rover")
                    angular_speed = self.reg_ang_vel
                    self.lin_vel = 0
                else:
                    print("angle from the right edge: " + str(ang))
                    #desired angle reached
                    if (abs(ang - center_of_frame) < headError):
                        break
                    #Move towards the desired angle (Not completely necessary)
                    if ((ang - center_of_frame) > 0):
                        #Turn right
                        angular_speed = self.reg_ang_vel
                        self.lin_vel = 0
                    else:
                        #Turn left
                        angular_speed = -self.reg_ang_vel
                        self.lin_vel = 0                        
                # Turn the rover until angle from the rover is oriented towards the tennis ball
                speed = [right_speed, left_speed]       #get rid after custom msg built########
                pub.publish(speed) 
                rate.sleep()
            print("distance to the ball: " + str(dis))

            #Drive the rover forward towards the ball
            speed = rover.speed
            pub.publish(speed)  ###new msgs
            rate.sleep()
        print("Arrived at the destination")

        #Stop the rover, onto the next marker
        speed = 0
        pub.publish(speed)          #create new msg######
        rate.sleep()

        #Stop for ~30 sec. just to show that the task has been accomplished
        time.sleep(time_delay_between_markers)

    print("task finished")
    # cleanup the camera and close any open windows
    camera.release()
    # out.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':

    #Time (sec) delay between each marker.
    time_delay_between_markers = 30

    #~5 meters within the gps coordinate
    xError = 0.00005
    yError = 0.00005

    #For CV: allowable angle of the tennis ball offset from the centre of the camera.
    error_range_of_tennisball = 10
    #Angle of the center of the camera.
    #Angle starts from the right side of the frame.
    center_of_frame = 30

    error_dist_of_tennisball_from_rover = 300

    # First Destination
    longitude = -79.403739
    latitude = 43.664809

    # The Destinations: (longitude, latitude)
    # You can add more markers onto the destination_markers array.
    destination_markers = [( longitude,  latitude), (1,1)]

    move_towards_the_ball(server, destination_markers, xError, yError, error_range_of_tennisball, center_of_frame, time_delay_between_markers, error_dist_of_tennisball_from_rover)

    #put a stop thing here#########



