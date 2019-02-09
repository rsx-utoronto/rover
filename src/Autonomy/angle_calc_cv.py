import numpy as np
def angle_cal(x, diameter, dis):
    #https://stackoverflow.com/questions/17499409/opencv-calculate-angle-between-camera-and-pixel

    #px coordinates of the center of the image
    x_center = 1280/2
    y_center = 960/2

    ball_diam_px = diameter

    #actual ball size in mm
    ball_diam_mm = 65

    #compute mm per px
    mm_px = ball_diam_mm/ball_diam_px

    #compute distance of ball from center in px
    diff = x_center - x #in px
    diff_mm = diff*mm_px

    angle = np.arctan(diff_mm/dis)
    angle = angle*180/np.pi
    return angle
