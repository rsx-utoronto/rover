from array import array
import copy
from glob import glob
from ntpath import join
from opcode import HAVE_ARGUMENT
from random import triangular
import time
import numpy as np
import pygame
import math
import sys
import threading
import rospy
from std_msgs.msg import String

# global variables

# joint angles
q1 = 0
q2 = 0
q3 = 0
q4 = 0
q5 = 0
q6 = 0
q7 = 0

angleSpeeds = {"q1": 1, "q2": 1, "q3": 1, "q4": 1, "q5": 1, "q6": 1, "q7": 1} # probably want to keep q5 and q6 the same
speedLevel = 0

joystick = None

# global constants

CODE_LOOPS_A_SEC = 1 # set to zero for max number of loops possible
BUTTON_NAMES = ["X", "CIRCLE", "TRIANGLE", "SQUARE", "L1", "R1", "L2-B", "R2-B", "SELECT", "START", "PLAY_STATION", "L3", "R3"]
JOYSTICK_AXES_NAMES = ["L-Right", "L-Down", "L2", "R-Right", "R-Down", "R2"]
SPEED_MULTIPLIER = 1.25

print(joystick.get_numbuttons)

