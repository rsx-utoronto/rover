#! /usr/bin/env python
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

CODE_LOOPS_A_SEC = 10 # set to zero for max number of loops possible
BUTTON_NAMES = ["X", "CIRCLE", "TRIANGLE", "SQUARE", "L1", "R1", "L2-B", "R2-B", "SELECT", "START", "PLAY_STATION", "L3", "R3", "D_UP", "D_DOWN", "D_RIGHT", "D_LEFT"]
JOYSTICK_AXES_NAMES = ["L-Right", "L-Down", "L2", "R-Right", "R-Down", "R2"]
SPEED_MULTIPLIER = 1.25

# Joystick and Button Functions 

def initializeJoystick(): # robot operated with a joystick - uses pygame library
    global joystick

    pygame.init()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print('Initialized joystick: %s' % joystick.get_name())
    
def getJoystickButtons(): # setting up the buttons
    global buttonNames

    pygame.event.pump() # allow pygame to handle internal actions, keep everything current
    
    buttons = {}
    
    for i in range(0, joystick.get_numbuttons()):
        buttons[BUTTON_NAMES[i]] = joystick.get_button(i) # get's if button is pressed

    return buttons

def getJoystickAxes(): # setting up the axes for the control stick
    global joystick
    global joystickAxesNames
    
    axes = {}
    
    pygame.event.pump()
    #Read input from the joystick       
    for i in range(0, joystick.get_numaxes()):
        axes[JOYSTICK_AXES_NAMES[i]] = joystick.get_axis(i)

        # set L2 and R2 input equal to zero if at/near rest position (they default to -1 which triggers movement) 
        if JOYSTICK_AXES_NAMES[i] == "L2" or JOYSTICK_AXES_NAMES[i] == "R2":
            if axes[JOYSTICK_AXES_NAMES[i]] <= -0.95: 
                axes[JOYSTICK_AXES_NAMES[i]] = 0
            else:
                axes[JOYSTICK_AXES_NAMES[i]] = 1
        else:
            if abs(axes[JOYSTICK_AXES_NAMES[i]]) <= 0.05: # nullifies movement caused by joystick being off centre
                axes[JOYSTICK_AXES_NAMES[i]] = 0
    
    return axes

# Angle Control

def setJointAngleLimits():
    global lim_q1_min, lim_q1_max, lim_q2_min, lim_q2_max, lim_q3_min, lim_q3_max, lim_q4_min, lim_q4_max, lim_q5_min, lim_q5_max, lim_q6_min, lim_q6_max
    global limitFlag
    
    limitFlag = True
    #In the order of q1lim to q6lim [min,max]
    lim_q1_min = -1401.0/(1680.0 * 60/24) *360
    lim_q1_max = 3001.0/(1680.0 * 60/24) *360
    lim_q2_min = -2159.0/(2048*4) *360
    lim_q2_max = 1957.0/(2048*4) *360
    lim_q3_min = -1181.0/(2048*4) *360
    lim_q3_max = 1264.0/(2048*4) *360
    lim_q4_min = -841.0/(1680) *360
    lim_q4_max = 841.0/(1680) *360
    lim_q5_min = -551.0/(1680) *360
    lim_q5_max = 401.0/(1680) *360
    lim_q6_min = -10000000000.0/(1680) *360
    lim_q6_max = 10000000000.0/(1680) *360

    return np.array([[lim_q1_min, lim_q1_max], [lim_q2_min, lim_q2_max], [lim_q3_min, lim_q3_max], [lim_q4_min, lim_q4_max], [lim_q5_min, lim_q5_max], [lim_q6_min, lim_q6_max]]) * math.pi/180

def sendAngleValues(qVect, start = 0): # sends a message with angle values - update number of encoder steps per 2*pi rotation
    global limitFlag

    # encoder steps per 2*pi rotation
    q1Steps = 1680.0 * 60/24
    q2Steps = 2048*4
    q3Steps = 2048*4
    q4Steps = 1680
    q5Steps = 1680
    q6Steps = 1680
    q7Steps = 1#26.9*64 # gripper
    # generate messages from qVect here q1String etc correspond to order in message, not exactly in qVect
    q1String = str( int(qVect[0] * q1Steps/(2*math.pi) ) )
    q2String = str( int(qVect[1] * q2Steps/(2*math.pi) ) )
    q3String = str( int(qVect[2] * q3Steps/(2*math.pi) ) )
    q4String = str( int(qVect[3] * q4Steps/(2*math.pi) ) )
    q5String = str( int(qVect[4] * q5Steps/(2*math.pi) ) )
    q6String = str( int(qVect[5] * q6Steps/(2*math.pi) ) )


    q7String = str( int(qVect[6] * q7Steps ) ) # gripper

    if limitFlag == True:
        command = 'p'
    elif limitFlag == False:
        command = 'f'
    message = command+" "+q1String+" "+q2String+" "+q3String+" "+q4String+" "+q5String+" "+q6String+" "+q7String
    print("message:",message)
    sendMessage(message)


# ROS Communication 

def sendMessage(message): # Tries to publish message to topic?
    global message_to_send
    #global conn
    global connFlag
    global THREAD_MODE_FLAG
    if connFlag == 1:
        if THREAD_MODE_FLAG:
            message_lock.acquire()
            try:
                message_to_send = message
            finally:
                message_lock.release()
        else:
            put_msg(message)

def put_msg(message): # Publishes message to the ROS topic
    #global conn
    global pub
    global connFlag
    try:
        if not rospy.is_shutdown(): # the code runs until the is_shutdown() flag is true, used to terminate the program properly
            #rospy.loginfo(message)
            pub.publish(message)
            rate.sleep()
        # print("The message to send is: {}".format(message))
        #conn.request("PUT", "/arm/"+message+"/")
        #conn.close()
    except rospy.ROSInterruptException as e:
        print("!!! ERROR in put_msg: " + str(e))
        time.sleep(0.1)

def messageThread(): #????? - from previous arm control code
    global message_to_send
    while True:
        message_lock.acquire()
        try:
            msg = message_to_send
        finally:
            message_lock.release()
        put_msg(msg)

# Control Functions
selectPressed = False
startPressed = False
trianglePressed = False


def manualControlMode(): # Manually Control the Arm
    global q1, q2, q3, q4, q5, q6, q7
    global turnedOn
    global angleSpeeds
    global speedLevel
    global selectPressed, startPressed
    global breakTrigger
    global qlim
    global savedJointAngle

    buttonPressed = getJoystickButtons()
    joystickAxes = getJoystickAxes()

    """
    Joint Control
    """

    q1 += angleSpeeds["q1"]*joystickAxes["R-Right"] # rotate base

    # move first actuator
    if buttonPressed["SQUARE"] == 1:
        q2 += angleSpeeds["q2"]
    if buttonPressed["CIRCLE"] == 1:
        q2 -= angleSpeeds["q2"]

    # move second actuator 
    q3 += angleSpeeds["q3"]*joystickAxes["R-Down"]

    # move worm gear joint
    q4 += angleSpeeds["q4"]*joystickAxes["L-Down"]
     
    # move the motors in opposite directions so the gears move together 
    q5 += angleSpeeds["q5"]*joystickAxes["L-Right"]
    q6 -= angleSpeeds["q6"]*joystickAxes["L-Right"]

    if buttonPressed["L1"] == 1:
        q5 += angleSpeeds["q5"]
        q6 -= angleSpeeds["q6"]
    
    if buttonPressed["R1"] == 1:
        q5 -= angleSpeeds["q5"]
        q6 += angleSpeeds["q6"]

    # open and close the gripper
    q7 += angleSpeeds["q7"]*joystickAxes["R2"]
    q7 -= angleSpeeds["q7"]*joystickAxes["L2"]


    if buttonPressed["L3"]:
        init_q2 = 1956.0/(2048*4)*360
        init_q3 = 1263.0/(2048*4)*360
        jointValues = np.array([0.0000000001,init_q2,init_q3,0.0000000001,0.0000000001,0.0000000001]) * math.pi/180 # degrees
        q1 = jointValues[0]
        q2 = jointValues[1]
        q3 = jointValues[2]
        q4 = jointValues[3]
        q5 = jointValues[4]
        q6 = jointValues[5]
        print("Arm reset to the Initial pose")

    if buttonPressed["R3"]:
        jointValues = np.array([0.0000000001,74.571,-39.836,0.0000000001,55.265,0.0000000001]) * math.pi/180
        q1 = jointValues[0]
        q2 = jointValues[1]
        q3 = jointValues[2]
        q4 = jointValues[3]
        q5 = jointValues[4]
        q6 = jointValues[5]
        print("Arm reset to the Forward pose")

    sendJointValues = [q1, q2, q3, q4, q5, q6, q7]

    # limits -> idk how accurate is cuz I adapted it from the existing code
    update = 1
    for i in range(6):
        if(sendJointValues[i] <= qlim[i][0] or sendJointValues[i] >= qlim[i][1] or abs(sendJointValues[i] - savedJointAngle[i]) > maxRot):
            update = 0
            print("Limit reached for joint: q" + str(i+1))
            break
    if(update == 0):
        for i in range(6):
            if sendJointValues[i] >= qlim[i][0] and sendJointValues[i] <= qlim[i][1]:
                continue
            elif sendJointValues[i] < qlim[i][0]: 
                sendJointValues[i] = qlim[i][0] # set to min limit if joint passes min
                # print(str(sendJointValues[i]) + " limit: " + str(qlim[i][0]))
            elif sendJointValues[i] > qlim[i][1]:
                sendJointValues[i] = qlim[i][1] # set to max limit if joint passes max

    savedJointAngle = sendJointValues
    sendAngleValues(sendJointValues)

    """
    Other Controls
    """
    
    if buttonPressed["PLAY_STATION"] == 1: # ends while loop in main
        turnedOn = False 
    
    if buttonPressed["SELECT"] == 1 and selectPressed == False: # decreases the speed of movement
        for joint in angleSpeeds:
            angleSpeeds[joint] /= SPEED_MULTIPLIER
        speedLevel -= 1
        selectPressed = True
    elif buttonPressed["SELECT"] == 0: # can't hold select to decrease speed
        selectPressed = False

    if buttonPressed["START"] == 1 and startPressed == False: # increases the speed of movement
        for joint in angleSpeeds:
            angleSpeeds[joint] *= SPEED_MULTIPLIER
        speedLevel += 1
        startPressed = True
    elif buttonPressed["START"] == 0: # can't hold select to decrease speed
        startPressed = False
    
# only run if this file is directly executed
if __name__ == "__main__":
    global savedJointAngles
    global savedGripperAngle
    global savedJointAngle
    global storageFile
    global k, t # velocity coefficients for translational and rotational motions
    global qlim
    global maxRot # determines max rotation by a joint per turn
    # declare limit angle variables as global to easily change between limited and limitless modes
    global lim_q1_min, lim_q1_max, lim_q2_min, lim_q2_max, lim_q3_min, lim_q3_max, lim_q4_min, lim_q4_max, lim_q5_min, lim_q5_max, lim_q6_min, lim_q6_max
    global limitFlag
    global turnedOn
    global breakTrigger

    # initializing ROS node
    global pub
    pub = rospy.Publisher('arm', String, queue_size=10)
    rospy.init_node('arm_talker', anonymous=True)
    global rate
    rate = rospy.Rate(10) # in Hz

    # for sending message as a thread
    global message_to_send
    global message_lock
    global THREAD_MODE_FLAG
    THREAD_MODE_FLAG = True
    message_to_send = ""
    message_lock = threading.Lock()


    # do networking if connFlag == 1, don't otherwise
    global connFlag
    connFlag = 1

    if THREAD_MODE_FLAG and connFlag == 1:
        message_worker = threading.Thread(target=messageThread)
        message_worker.start()

    maxRot = 2*math.pi*10000/360 
    k = 0.6
    t = 0.03

    serverIP = '192.168.0.3'
    serverHttpPort = '8080'
    #global conn
    #conn = httplib.HTTPConnection(serverIP+":"+serverHttpPort)

    # choose between import and non-import modes
    if len(sys.argv)==2 and sys.argv[1] == 'import':
        angleFile = open('src/ArmControl/savedJointAngles.txt','r')
        angles = angleFile.read()
        angleFile.close()
        angles = angles.strip()
        angles = angles.strip('[]')
        angles = angles.split(',')
        for i in range( len(angles) ):
            angles[i] = float(angles[i])
        #pass #( angles )
        savedJointAngles = angles

        gripperFile = open('src/ArmControl/src/savedGripperAngle.txt','r')
        gripper = gripperFile.read()
        gripperFile.close()
        gripper = gripper.strip()
        gripper = float(gripper)
        savedGripperAngle = gripper

    else:
        init_q2 = 1956.0/(2048*4)*360
        init_q3 = 1263.0/(2048*4)*360
        savedJointAngles = np.array([0,init_q2,init_q3,0,0,0]) * math.pi/180 # degrees
        savedGripperAngle = 0

    qlim = setJointAngleLimits()
    #qlim = np.array([[-18000, 18000], [-18000, 18000], [-18000, 18000], [-18000, 18000], [-18000, 18000], [-18000, 18000]]) * math.pi/180
    #print(qlim)

    savedJointAngle = [q1, q2, q3, q4, q5, q6, q7]

    initializeJoystick()

    breakTrigger = False #GET THE BREAK TRIGGER FROM SOMEWHERE
    turnedOn = True
    while turnedOn:

        # frequency of the model loop in Hz
        frequency = 20
        timeDelay =  1.0/frequency
        #pass #(timeDelay)


        if turnedOn:
            if breakTrigger:
                print("Break triggered")
            else:
                manualControlMode()
        
        if CODE_LOOPS_A_SEC != 0:
            time.sleep(0.05)

    print("Shutting Down The Arm!")
    