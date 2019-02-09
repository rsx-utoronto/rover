# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import time
from distance_calc_cv import find_distance_given_diameter
from angle_calc_cv import angle_cal
#import publishingNode.py

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=50,
help="max buffer size")
args = vars(ap.parse_args())

#FOCAL_LENGTH = 4.15 # sally's phone in mm
FOCAL_LENGTH = 3.175; #computed from the vesky's camera matrix -- in mm (see testing.py for computation)
BALL_DIMENSION = 65 # sally measured this

# ------- camera matrix comes from calibration algorithm from distance_calc_cv.py (find_intrinsic_matrix function)
# or found in camera's doc ------------------
#kmtx = np.load('C:/Users/rache/PycharmProjects/rsx/k_matrix.npy') #reading in the camera matrix from sally's iphone
kmtx = np.load('v_matrix.npy') #reading in the camera matrix, from the vesky camera
calib_list = kmtx[0, 0], kmtx[1, 1], kmtx[0, 2], kmtx[1, 2]
#----------------------------------------------------

# the avi format may not work depending on computer, also its super big
# also pls input the proper size for each video frame at the end
out = cv2.VideoWriter('./output.avi', -1, 25.0, (960,544))

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points

#------ important!!! (if ball is not properly detected, change these)-----
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
# ------------------------------------------------------------------------

pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference to the webcam
if not args.get("video", False):
	camera = cv2.VideoCapture(0)
	#camera = cv2.VideoCapture('IMG_3351.MOV')

# otherwise, grab a reference to the video file
else:
	camera = cv2.VideoCapture(args["video"])
frameCount = 0
# keep looping
while True:

	NUM_FRAMES = 10000

	# Comute for every x frames
	if (frameCount != NUM_FRAMES):
		frameCount += 1
		continue
	# grab the current frame
	# frame is the next frame, grabbed is bool showing if there is a next frame
	start = time.time()
	(grabbed, frame) = camera.read()

	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	if args.get("video") and not grabbed:
		break

	# resize the frame, blur it, and convert it to the HSV
	# color space

	# ------ optional (works without them) ------
	# frame = imutils.resize(frame, width=800)
	# blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	# ----------------------------------------
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid

		# can also set a minimum limit on the radius, when ball is far away (if radius > 20 do the following)
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		cv2.circle(frame, (int(x), int(y)), int(radius),
				   (255, 0, 0), 2)

		# calculate distance from the file distance_calc_cv.py
		# works only with known camera matrix (one that was calibrated with size or resolution the same as size here)
		dis = find_distance_given_diameter(FOCAL_LENGTH, BALL_DIMENSION, calib_list, frame,radius*2)
		font = cv2.FONT_HERSHEY_SIMPLEX
		cv2.putText(frame, "{0:.2f}".format(dis/10)+'inches', (center[0]+5,center[1]+5), font, 0.8, (0, 255, 0), 2)

		#calculate the angle displacement from the center of the image
		angle = angle_cal(x,radius*2, dis);
		font = cv2.FONT_HERSHEY_SIMPLEX
		#cv2.putText(frame, "{0:.2f}".format(angle) + 'degrees', (center[0] + 5, center[1] + 10), font, 0.8, (0, 255, 0),2)

	# update the points queue
	pts.appendleft(center)

	# loop over the set of tracked points
	for i in range(1, len(pts)):
		# if either of the tracked points are None, ignore them
		if pts[i - 1] is None or pts[i] is None:
			continue

		# otherwise, compute the thickness of the line and
		# draw the connecting lines
		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 1.5)
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

	end = time.time()
	#print(end - start)
	# show the frame to our screen
	# out.write(frame)
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	out.write(frame)
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
	
	#publishingNode.talker(dis, angle)

# cleanup the camera and close any open windows
camera.release()
out.release()
cv2.destroyAllWindows()
	
