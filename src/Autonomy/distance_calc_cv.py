# method ref: https://stackoverflow.com/questions/14038002/opencv-how-to-calculate-distance-between-camera-and-object-using-image


import numpy as np
import cv2
import glob
import imutils

def find_distance_given_diameter(focal_lenth,ball_diameter_real,calib_set,frame,diameter):
    [fx, fy, px, py] = calib_set

    # find px/mm on image sensor -> m=(fx+fy)/2/f
    px_mm_sensor = (fx + fy) / 2 / focal_lenth
   # px_mm_sensor_video = 960*px_mm_sensor/4032
    px_mm_sensor_video = 960*px_mm_sensor/1280 #computation for the veskys camera


    # find size of object s in px
    ball_diameter = diameter

    #print('ball diameter on image %f', ball_diameter)

    # s px/(m px/mm) -> size of object on image sensor im mm
    ball_diameter_sensor = diameter/px_mm_sensor_video

    # distance_mm = object_real_world_mm * focal-length_mm / object_image_sensor_mm
    distance = ball_diameter_real * focal_lenth / ball_diameter_sensor

    return distance


def angle_cal():
    #https://stackoverflow.com/questions/17499409/opencv-calculate-angle-between-camera-and-pixel
    pass



def find_distance(focal_lenth,ball_diameter_real,calib_set,frame):

    # feed into opencv calibration, get intrinsic cam matrix -> fx,fy, px,py
    # [fx, fy, px, py] = find_intrinsic_matrix(calib_imgset)
    [fx, fy, px, py] = calib_set

    # find px/mm on image sensor -> m=(fx+fy)/2/f
    px_mm_sensor = (fx + fy) / 2 / focal_lenth

    # find size of object s in px
    ball_diameter = find_ball_diameter(frame)

    print('ball diameter on image %f' ,ball_diameter)

    # s px/(m px/mm) -> size of object on image sensor im mm
    ball_diameter_sensor = ball_diameter / px_mm_sensor

    # distance_mm = object_real_world_mm * focal-length_mm / object_image_sensor_mm
    distance = ball_diameter_real * focal_lenth / ball_diameter_sensor

    return distance

# detect and find diameter of ball
def find_ball_diameter(frame):

    # resize the frame, blur it, and convert it to the HSV
    # color space
    # frame = imutils.resize(frame, width=800)
    # blurred = cv2.GaussianBlur(frame, (11, 11), 0)

    print('image shape is %s '%(frame.shape,))

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    greenLower = (20, 80, 80)  # (0-180,0-255,0-255)
    greenUpper = (100, 255, 255)
    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    cv2.imshow("mask before de-noise", mask)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cv2.imwrite('mask.jpg',mask)
    cv2.imshow("mask", mask)
    cv2.waitKey(0)

    print('masked')
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    # print('contour length %d', len(cnts))
    # print(cnts)
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        print('centre')
        print(center)
        cv2.circle(frame, (int(x), int(y)), int(radius),
                   (255, 0, 0), 10)
        cv2.circle(frame, center, 5, (0, 0, 255), -1)
        # only proceed if the radius meets a minimum size
        cv2.imwrite('./test_balls2/circle1.jpg', frame)
        return radius*2
    else:
        return -1

# calibration with 20 checkboard images taken from different angles
# image has to be purely black and white, background of checkerboard has to be white! -- important!!!!
def find_intrinsic_matrix(images):
    # ref: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html


    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    print("enter func")
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((7 * 7, 3), np.float32)

    # grid size depends on checkboard
    objp[:, :2] = np.mgrid[0:7, 0:7].T.reshape(-1, 2)  # to unspecified value x 2

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    for fname in images:
        print("new iter")
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,7), None)
        # print(corners)
        if corners is not None:
            print(len(corners))
        else:
            print('0-----' + fname)
        print(ret)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (7, 7), corners2, ret)
            cv2.imshow('img', img)
            save_to_folder = 'C:/git/rover/cv/saved_img'

            cv2.imwrite(save_to_folder+'/'+fname.split('\\')[1],img)
        # cv2.waitKey(50000)

        # print(imgpoints)

    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print('camera param =')
    print(mtx)
    np.save('C:/git/rover/cv/k_matrix.npy',mtx)
    np.save('C:/git/rover/cv/rvecs.npy',rvecs)
    np.save('C:/git/rover/cv/tvecs.npy',tvecs)

    fx,fy,px,py = mtx[0,0],mtx[1,1],mtx[0,2],mtx[1,2]
    return  [fx,fy,px,py]


if __name__ == "__main__":
    # focal lengthe of this camera
    # FOCAL_LENGTH = 4.3  # mm, for testing purpose with a phone rn.
    FOCAL_LENGTH = 4.15 #sally's phone in mm
    BALL_DIMENSION = 65

    # find native resolution of photos taken - only for comparing photo at diff resolution from native

    # size of obj in real world in mm, diameter in this case
    ball_diameter_real = ""

    # take about 2 dozen photos,
    CALIB_IMGSET = glob.glob('C:/git/rover/cv/calic_img2cut/*.jpg')

    # input image to find distance, frame past by ball_tracking.py or newly taken
    FRAME = ""

    # [fx, fy, px, py] = find_intrinsic_matrix(CALIB_IMGSET)
    # find_intrinsic_matrix(CALIB_IMGSET)
    # print([fx, fy, px, py])

    frame = cv2.imread('./test_balls2/test1_ed.jpg')
    kmtx = np.load('C:/git/rover/cv/k_matrix.npy')
    calib_list = kmtx[0, 0], kmtx[1, 1], kmtx[0, 2], kmtx[1, 2]
    print(find_distance(FOCAL_LENGTH, BALL_DIMENSION, calib_list, frame))