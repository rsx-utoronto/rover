#! /usr/bin/env python
# imports
import copy
import time
import openravepy
import numpy as np
import pygame
import math
import httplib
import sys
import threading
import rospy
from std_msgs.msg import String

gripper_encoder_missing = 1



def put_msg(message):
    #global conn
    global pub
    global connFlag
    try:
        if not rospy.is_shutdown(): # the code runs until the is_shutdown() flag is true, used to terminate the program properly
            #rospy.loginfo(message)
            pub.publish(message)
            rate.sleep()
        # print ("The message to send is: {}".format(message))
        #conn.request("PUT", "/arm/"+message+"/")
        #conn.close()
    except rospy.ROSInterruptException as e:
        print "!!! ERROR in put_msg:"
        print e
        time.sleep(0.1)


def messageThread():
    global message_to_send
    while True:
        message_lock.acquire()
        try:
            msg = message_to_send
        finally:
            message_lock.release()
        put_msg(msg)


def homogenousTransform(dhVector): # set up a homogenous matrix given the parameters that describe a joint / the transformation
    a = dhVector[0]
    alpha = dhVector[1]
    d = dhVector[2]
    theta = dhVector[3]
    homTransMatrix = [ [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
                       [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
                       [0, math.sin(alpha), math.cos(alpha), d],
                       [0, 0, 0, 1] ]
    return homTransMatrix


def forwardKinematics(dhTable): # use the dh table to perform forward kinematics - outputs the end position
    A1 = np.matrix( homogenousTransform(dhTable[0]) )
    A2 = np.matrix( homogenousTransform(dhTable[1]) )
    A3 = np.matrix( homogenousTransform(dhTable[2]) )
    A4 = np.matrix( homogenousTransform(dhTable[3]) )
    A5 = np.matrix( homogenousTransform(dhTable[4]) )
    A6 = np.matrix( homogenousTransform(dhTable[5]) )
    fullHomTransMatrix = A1*A2*A3*A4*A5*A6
    #print( fullHomTransMatrix.tolist() )
    return fullHomTransMatrix.tolist()


def inverseKinematics(dhTable, homTransMatrix): # perform inverse kinematics
# take in a input position we want the end effector to be in, calculates the different joint variables q1 to q6 and calculates what those are in order to orient the arm in that position
# updates DH table and sent out as output
# all DH tables are instructions for microcontrollers
# drive motors to exact position that we want
    try:
        d1 = dhTable[0][2]
        a2 = dhTable[1][0]
        d4 = dhTable[3][2]
        d6 = dhTable[5][2]

        #print("HomTransMatrix: ")
        #print(homTransMatrix)
        
        Rd = np.matrix( [ homTransMatrix[0][:3], homTransMatrix[1][:3], homTransMatrix[2][:3] ] )
        #print("Rd: ")
        #print(Rd)
        od = np.matrix( [ homTransMatrix[0][3], homTransMatrix[1][3], homTransMatrix[2][3] ] ).transpose()
        #print("od: ")
        #print(od)

        arD6 = np.matrix( [ 0,0,d6] ).transpose()
        #print("arD6: ")
        #print(arD6)
        #print(arD6.transpose())
        #print(Rd)
        oc = od - Rd * arD6
        #print(np.dot(Rd,arD6))
        #print(np.dot(Rd,arD6.transpose()))
        #print("oc: ")
        #print(oc)

        xc = oc.tolist()[0][0]
        yc = oc.tolist()[1][0]
        zc = oc.tolist()[2][0]

        q1 = math.atan2(yc, xc)
        
        Dtemp = ( xc**2 + yc**2 + (zc-d1)**2 - a2**2 - d4**2 )/( 2*a2*d4 )
        if abs(Dtemp) > 1:
            print( 'Can not reach {}, {}, {}'.format( homTransMatrix[0][3], homTransMatrix[1][3], homTransMatrix[2][3] ) )
            return dhTable
        
        q3 = math.atan2( Dtemp, math.sqrt(1 - Dtemp**2) ) # assumes we chose to always have 'elbow up' solution
        
        q2 = math.atan2( zc - d1, math.sqrt(xc**2 + yc**2) ) - math.atan2( d4*math.sin(q3-math.pi/2), a2+d4*math.cos(q3-math.pi/2) )

        # start updating dhTable. Update with first 3 joints
        updatedDHTable = copy.deepcopy(dhTable)
        updatedDHTable[0][3] = q1
        updatedDHTable[1][3] = q2
        updatedDHTable[2][3] = q3
        
        H01 = np.matrix( homogenousTransform(updatedDHTable[0]) )
        H12 = np.matrix( homogenousTransform(updatedDHTable[1]) )
        H23 = np.matrix( homogenousTransform(updatedDHTable[2]) )
        H03 = (H01 * H12 * H23).tolist()
        R03 = np.matrix( [H03[0][:3], H03[1][:3], H03[2][:3]] )
        #print("R03: ")
        #print(R03)
        #print(R03)
        #print(R03.transpose())
        R36 = R03.transpose() * Rd
        #print("R36: ")
        #print(R36)
        #print( np.dot(R03, Rd) )
        #print( np.dot(R03.transpose(), Rd) )

        q4 = math.atan2( R36.tolist()[1][2], R36.tolist()[0][2] )
        q5 = math.atan2( math.sqrt(1 - R36.tolist()[2][2]**2), R36.tolist()[2][2] )
        q6 = math.atan2( R36.tolist()[2][1], -R36.tolist()[2][0] )

        # updating DH table with the wrist stuff
        updatedDHTable[3][3] = q4
        updatedDHTable[4][3] = q5
        updatedDHTable[5][3] = q6
        
        return updatedDHTable
    except:
        return dhTable


def inverseKinematicsPositional(dhTable, homTransMatrix, rotationVector):
# keeps arm still
# ex. keep end effector at a position, but just move a few inches
    try:
        d1 = dhTable[0][2]
        a2 = dhTable[1][0]
        d4 = dhTable[3][2]
        d6 = dhTable[5][2]

        #print("HomTransMatrix: ")
        #print(homTransMatrix)
        
        Rd = np.matrix( [ homTransMatrix[0][:3], homTransMatrix[1][:3], homTransMatrix[2][:3] ] )
        #print("Rd: ")
        #print(Rd)
        od = np.matrix( [ homTransMatrix[0][3], homTransMatrix[1][3], homTransMatrix[2][3] ] ).transpose()
        #print("od: ")
        #print(od)

        arD6 = np.matrix( [ 0,0,d6] ).transpose()
        #print("arD6: ")
        #print(arD6)
        #print(arD6.transpose())
        #print(Rd)
        oc = od - Rd * arD6
        #print(np.dot(Rd,arD6))
        #print(np.dot(Rd,arD6.transpose()))
        #print("oc: ")
        #print(oc)

        xc = oc.tolist()[0][0]
        yc = oc.tolist()[1][0]
        zc = oc.tolist()[2][0]

        q1 = math.atan2(yc, xc)
        
        Dtemp = ( xc**2 + yc**2 + (zc-d1)**2 - a2**2 - d4**2 )/( 2*a2*d4 )
        if abs(Dtemp) > 1:
            print( 'Can not reach {}, {}, {}'.format( homTransMatrix[0][3], homTransMatrix[1][3], homTransMatrix[2][3] ) )
            return dhTable
        
        q3 = math.atan2( Dtemp, math.sqrt(1 - Dtemp**2) ) # assumes we chose to always have 'elbow up' solution
        
        q2 = math.atan2( zc - d1, math.sqrt(xc**2 + yc**2) ) - math.atan2( d4*math.sin(q3-math.pi/2), a2+d4*math.cos(q3-math.pi/2) )

        # start updating dhTable. Update with first 3 joints
        updatedDHTable = copy.deepcopy(dhTable)
        updatedDHTable[0][3] = q1
        updatedDHTable[1][3] = q2
        updatedDHTable[2][3] = q3
        updatedDHTable[3][3] = rotationVector[1]
        updatedDHTable[4][3] = rotationVector[0]
        updatedDHTable[5][3] = rotationVector[2]
        
        return updatedDHTable
    except:
        return dhTable


def updateHomTransMatrix(homTransMatrix, DHTable, translationVector, rotationVector):
#updates the homogenous matrix after it gets to its end position
    global ikType
    updatedHomTransMatrix = copy.deepcopy( homTransMatrix )
    global k, t
    for ind in range(3):
        translationVector[ind] = k * translationVector[ind]
        rotationVector[ind] = t * rotationVector[ind]

    ### TYPE0 fancy motions, relative to the camera reference frame
    if ikType == 0:
        ## update movement in the end effector camera frame
        x0 = np.matrix( [1, 0, 0] ).transpose()
        y0 = np.matrix( [0, 1, 0] ).transpose()
        z0 = np.matrix( [0, 0, 1] ).transpose()

        # get z4
        A1 = np.matrix( homogenousTransform(DHTable[0]) )
        A2 = np.matrix( homogenousTransform(DHTable[1]) )
        A3 = np.matrix( homogenousTransform(DHTable[2]) )
        A4 = np.matrix( homogenousTransform(DHTable[3]) )
        H04 = A1*A2*A3*A4
        R04 = H04[:3,:3]
        z4 = R04 * z0
        
        #get z6
        R06 = np.matrix( [ updatedHomTransMatrix[0][:3], updatedHomTransMatrix[1][:3], updatedHomTransMatrix[2][:3] ] )
        z6 = R06 * z0

        # define the end effector camera frame
        z0_tilda = z6.transpose().tolist()[0]
        y0_tilda = z4.transpose().tolist()[0]
        x0_tilda = np.cross(y0_tilda,z0_tilda)
        x0_tildaNorm = np.linalg.norm(x0_tilda)
        for i in range(len(z0_tilda)):
            x0_tilda[i] = x0_tilda[i]/x0_tildaNorm

        #print(x0_tilda)
        #print(y0_tilda)
        #print(z0_tilda)

        # construct R00_tilda
        x0 = x0.transpose().tolist()[0]
        y0 = y0.transpose().tolist()[0]
        z0 = z0.transpose().tolist()[0]
        R00_tilda = np.matrix( [ [np.dot(x0_tilda,x0), np.dot(y0_tilda,x0), np.dot(z0_tilda,x0)], 
                                [np.dot(x0_tilda,y0), np.dot(y0_tilda,y0), np.dot(z0_tilda,y0)], 
                                [np.dot(x0_tilda,z0), np.dot(y0_tilda,z0), np.dot(z0_tilda,z0)] ] )
        # update translation
        xMovement = R00_tilda * np.matrix( [translationVector[2], 0, 0] ).transpose()
        yMovement = R00_tilda * np.matrix( [0, -translationVector[1], 0] ).transpose()
        zMovement = R00_tilda * np.matrix( [0, 0, translationVector[0]] ).transpose()
        # update rotation
        Rx = np.matrix( [ [1, 0, 0], 
                            [0, math.cos(rotationVector[1]), -math.sin(rotationVector[1])], 
                            [0, math.sin(rotationVector[1]), math.cos(rotationVector[1])] ] )
        Ry = np.matrix( [ [math.cos(rotationVector[0]), 0, math.sin(rotationVector[0])], 
                            [0, 1, 0], 
                            [-math.sin(rotationVector[0]), 0, math.cos(rotationVector[0])] ] )
        Rz = np.matrix( [ [math.cos(rotationVector[2]), -math.sin(rotationVector[2]), 0], 
                            [math.sin(rotationVector[2]), math.cos(rotationVector[2]), 0], 
                            [0, 0, 1] ] )
        Rx0_tilda = R00_tilda * Rx * R00_tilda.transpose()
        Ry0_tilda = R00_tilda * Ry * R00_tilda.transpose()
        Rz0_tilda = R00_tilda * Rz * R00_tilda.transpose()
        # update homogenous transform matrix
        updatedHomTransMatrix[0][3] += (xMovement.tolist()[0][0] + yMovement.tolist()[0][0] + zMovement.tolist()[0][0])
        updatedHomTransMatrix[1][3] += (xMovement.tolist()[1][0] + yMovement.tolist()[1][0] + zMovement.tolist()[1][0])
        updatedHomTransMatrix[2][3] += (xMovement.tolist()[2][0] + yMovement.tolist()[2][0] + zMovement.tolist()[2][0])
        #rotation is happening in the order: around z -> around y -> around x
        R06_updated = (Rx0_tilda * Ry0_tilda * Rz0_tilda * R06).tolist()
        #print R06_updated

    ### TYPE 1 fancy motions (in the tip reference frame)
    elif ikType == 1:
        # update translation 
        R06 = np.matrix( [ updatedHomTransMatrix[0][:3], updatedHomTransMatrix[1][:3], updatedHomTransMatrix[2][:3] ] )
        # forward-backward movement update
        xMovement = R06 * np.matrix( [0, 0, translationVector[0]] ).transpose()
        updatedHomTransMatrix[0][3] += xMovement.tolist()[0][0]
        updatedHomTransMatrix[1][3] += xMovement.tolist()[1][0]
        updatedHomTransMatrix[2][3] += xMovement.tolist()[2][0]
        # left-right movement update
        yMovement = R06 * np.matrix( [0, -translationVector[1], 0] ).transpose() # put "-" for the right movement direction
        updatedHomTransMatrix[0][3] += yMovement.tolist()[0][0]
        updatedHomTransMatrix[1][3] += yMovement.tolist()[1][0]
        updatedHomTransMatrix[2][3] += yMovement.tolist()[2][0]
        # up-down movement update
        zMovement = R06 * np.matrix( [translationVector[2], 0, 0] ).transpose()
        updatedHomTransMatrix[0][3] += zMovement.tolist()[0][0]
        updatedHomTransMatrix[1][3] += zMovement.tolist()[1][0]
        updatedHomTransMatrix[2][3] += zMovement.tolist()[2][0]

        # update rotation (in the tip reference frame)
        # basic rotations
        Rx = np.matrix( [ [1, 0, 0], 
                            [0, math.cos(rotationVector[1]), -math.sin(rotationVector[1])], 
                            [0, math.sin(rotationVector[1]), math.cos(rotationVector[1])] ] )
        Ry = np.matrix( [ [math.cos(rotationVector[0]), 0, math.sin(rotationVector[0])], 
                            [0, 1, 0], 
                            [-math.sin(rotationVector[0]), 0, math.cos(rotationVector[0])] ] )
        Rz = np.matrix( [ [math.cos(rotationVector[2]), -math.sin(rotationVector[2]), 0], 
                            [math.sin(rotationVector[2]), math.cos(rotationVector[2]), 0], 
                            [0, 0, 1] ] )
        # update rotation matrix
        # rotation is happening in the order: around x -> around y -> around z
        R06_updated = (R06 * Rx * Ry * Rz).tolist()
        #print R06_updated


    # ###!!! move the end effector to the specific predAll DOFs modeefined orientation, based on the predefined rotation matrices
    # buttons = getJoystickButtons()
    # #print buttons

    # turn to "Down" pose
    # if buttons[18] == 1:
    #     R06_updated = np.matrix( [ [1, 0, 0],
    #                                 [0, -1, 0],
    #                                 [0, 0, -1] ] ).tolist()
    # # turn to "Forward" pose
    # elif buttons[17] == 1:
    #     R06_updated = np.matrix( [ [0, 0, 1],
    #                                 [0, -1, 0],
    #                                 [1, 0, 0] ] ).tolist()
    # # turn to "Left" orientation
    # elif buttons[21] == 1:
    #     R06_updated = np.matrix( [ [0, -1, 0],
    #                                 [0, 0, -1],
    #                                 [1, 0, 0] ] ).tolist()
    # # turn to "Right" orientation
    # elif buttons[21] == 1:
    #     R06_updated = np.matrix( [ [0, 1, 0],
    #                                 [0, 0, 1],
    #                                 [1, 0, 0] ] ).tolist()
    # turn to "Left-Forward" orientation
    # elif buttons[19] == 1:
    #     R06_updated = np.matrix( [ [0, -1/math.sqrt(2), 1/math.sqrt(2)],
    #                                 [0, -1/math.sqrt(2), -1/math.sqrt(2)],
    #                                 [1, 0, 0] ] ).tolist()
    # # turn to "Right-Forward" orientation
    # elif buttons[20] == 1:
    #     R06_updated = np.matrix( [ [0, 1/math.sqrt(2), 1/math.sqrt(2)],
    #                                 [0, -1/math.sqrt(2), 1/math.sqrt(2)],
    #                                 [1, 0, 0] ] ).tolist()
    # # turn to "Forward-Down" orientation
    # elif buttons[21] == 1:
    #     R06_updated = np.matrix( [ [1/math.sqrt(2), 0, 1/math.sqrt(2)],
    #                                 [0, -1, 0],
    #                                 [1/math.sqrt(2), 0, -1/math.sqrt(2)] ] ).tolist()
    # # turn to "Forward-Up" orientation
    # elif buttons[21] == 1:
    #     R06_updated = np.matrix( [ [-1/math.sqrt(2), 0, 1/math.sqrt(2)],
    #                                 [0, -1, 0],
    #                                 [1/math.sqrt(2), 0, 1/math.sqrt(2)] ] ).tolist()
    # # turn to "Left-Down" orientation
    # elif buttons[21] == 1:
    #     R06_updated = np.matrix( [ [1, 0, 0],
    #                                 [0, -1/math.sqrt(2), -1/math.sqrt(2)],
    #                                 [0, 1/math.sqrt(2), -1/math.sqrt(2)] ] ).tolist()
    # # turn to "Right-Down" orientation
    # elif buttons[21] == 1:
    #     R06_updated = np.matrix( [ [1, 0, 0],
    #                                 [0, -1/math.sqrt(2), 1/math.sqrt(2)],
    #                                 [0, -1/math.sqrt(2), -1/math.sqrt(2)] ] ).tolist()
    # # turn to "Forward-Up-Left" orientation
    # elif buttons[21] == 1:
    #     R06_updated = np.matrix( [ [-1/math.sqrt(3), -1/math.sqrt(3), 1/math.sqrt(3)],
    #                                 [1/math.sqrt(3), -1/math.sqrt(3), -1/math.sqrt(3)],
    #                                 [1/math.sqrt(3), -1/math.sqrt(3), 1/math.sqrt(3)] ] ).tolist()
    # # turn to "Forward-Up-Right" orientation
    # elif buttons[21] == 1:
    #     R06_updated = np.matrix( [ [-1/math.sqrt(3), 1/math.sqrt(3), 1/math.sqrt(3)],
    #                                 [1/math.sqrt(3), -1/math.sqrt(3), 1/math.sqrt(3)],
    #                                 [1/math.sqrt(3), 1/math.sqrt(3), 1/math.sqrt(3)] ] ).tolist()
    # # turn to "Forward-Down-Left" orientation
    # elif buttons[21] == 1:
    #     R06_updated = np.matrix( [ [1/math.sqrt(3), -1/math.sqrt(3), 1/math.sqrt(3)],
    #                                 [-1/math.sqrt(3), -1/math.sqrt(3), -1/math.sqrt(3)],
    #                                 [1/math.sqrt(3), 1/math.sqrt(3), -1/math.sqrt(3)] ] ).tolist()
    # # turn to "Forward-Down-Right" orientation
    # elif buttons[21] == 1:
    #     R06_updated = np.matrix( [ [1/math.sqrt(3), 1/math.sqrt(3), 1/math.sqrt(3)],
    #                                 [1/math.sqrt(3), -1/math.sqrt(3), 1/math.sqrt(3)],
    #                                 [1/math.sqrt(3), 1/math.sqrt(3), -1/math.sqrt(3)] ] ).tolist()
    # # turn to "Up" orientation
    # elif buttons[21] == 1:
    #     R06_updated = np.matrix( [ [-1, 0, 0],
    #                                 [0, -1, 0],
    #                                 [0, 0, 1] ] ).tolist()
    # # turn to "Left-Up" orientation
    # elif buttons[21] == 1:
    #     R06_updated = np.matrix( [ [-1, 0, 0],
    #                                 [0, -1/math.sqrt(2), -1/math.sqrt(2)],
    #                                 [0, -1/math.sqrt(2), 1/math.sqrt(2)] ] ).tolist()
    # # turn to "Right-Up" orientation
    # elif buttons[21] == 1:
    #     R06_updated = np.matrix( [ [-1, 0, 0],
    #                                 [0, -1/math.sqrt(2), 1/math.sqrt(2)],
    #                                 [0, 1/math.sqrt(2), 1/math.sqrt(2)] ] ).tolist()
   
    #print R06_updated
    # update homTrans matrix
    for i in range(3):
        for j in range(3):
            updatedHomTransMatrix[i][j] = R06_updated[i][j] 
    #print updatedHomTransMatrix
    
    return updatedHomTransMatrix


def updateHomTransMatrixPositional(homTransMatrix, DHTable, translationVector, rotationVector):
# updates the homogenous matrix after it gets to its end position, positional joints
    updatedHomTransMatrix = copy.deepcopy( homTransMatrix )
    global k, t
    for ind in range(3):
        translationVector[ind] = k * translationVector[ind]
        rotationVector[ind] = t * rotationVector[ind]
        # update translational part
        updatedHomTransMatrix[ind][3] += translationVector[ind] 
    
    return updatedHomTransMatrix


def initializeJoystick():
# robot operated with a joystick - uses pygame library
    pygame.init()
    global joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print('Initialized joystick: %s' % joystick.get_name())


def setupVisualEnv():
# set up a visual environment that simulates arm in real time to give window that tells you arm is moving correctly
# set up 3d model that moves in real time like the arm and shows us visual of how arm is moving throughout different tasks
    env = openravepy.Environment()
    env.Load('src/ArmControl/environment.xml')
    env.SetViewer('qtcoin')
    viewer = env.GetViewer()
    global robot
    robot = env.GetRobots()[0]
    global initAngles
    # if want to modify q4 and q6, reverse the sign from "angle" to "-angle"
    # needed specifically for visualizer
    global savedJointAngles
    copySavedJointAngles = savedJointAngles[:]
    initAngles = [0,0,-math.pi/2,0,0,0]

    setupAngles = []
    for i in range(len(initAngles)):
        setupAngles.append( copySavedJointAngles[i]+initAngles[i] )
        
    robot.SetActiveDOFValues(setupAngles)
    #print( robot.GetActiveDOFValues() )


def getJoystickButtons(): # setting up the buttons
    pygame.event.pump() # allow pygame to handle internal actions, keep everything current
    
    buttons = []
    for i in range(0, joystick.get_numbuttons()):
        button = joystick.get_button(i)
        buttons.append(button)
    #print(buttons)
    return buttons


def sendAngleValues(qVect, start = 0): # sends a message with angle values?
    global limitFlag
    global modeOfOperation

    if modeOfOperation == 4:
        
        # generate messages from qVect here q1String etc correspond to order in message, not exactly in qVect
        q1String = str( qVect[0] )
        q2String = str( qVect[1] )
        q3String = str( qVect[2] )
        q4String = str( qVect[3] )
        q5String = str( qVect[4] )
        q6String = str( qVect[5] )
        q7String = str( qVect[6] ) # gripper
        
        command = 'm'
        message = command+" "+q1String+" "+q2String+" "+q3String+" "+q4String+" "+q5String+" "+q6String+" "+q7String
        #print message
        sendMessage(message)
    else:   
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

        if gripper_encoder_missing:
            buttons = getJoystickButtons()
            if buttons[22] == 1:
                gripperSpeed = -255
            elif buttons[25] == 1:
                gripperSpeed = 255
            else:
                gripperSpeed = 0
            q7String = str (gripperSpeed)
        else:
            q7String = str( int(qVect[6] * q7Steps ) ) # gripper

        if limitFlag == True:
            command = 'p'
        elif limitFlag == False:
            command = 'f'
        message = command+" "+q1String+" "+q2String+" "+q3String+" "+q4String+" "+q5String+" "+q6String+" "+q7String
        sendMessage(message)

    # emergency stop
    if buttons[1] == 1:
        message = 'e'
        sendMessage(message)
    # resume from emergency stop
    if buttons[23] == 1:
        message = 'c'
        sendMessage(message)
    # starting position calibration
    if buttons[9] == 1:
        message = 's'
        sendMessage(message)

    

def getJoystickAxes(): # setting up the axes for the control stick
    out = [0,0,0,0,0,0]
    it = 0 # iterator
    pygame.event.pump()
    #Read input from the joystick       
    for i in range(0, joystick.get_numaxes()):
        out[i] = joystick.get_axis(i)
    return out


def getJoystickDirection(): # read the position of the stick
    global modeOfMovement
    global savedJointAngles
    global modeOfOperation

    joystickValues = getJoystickAxes()
    #print("Joystick direction values: {}".format(joystickValues))

    if modeOfMovement == 0:
        print("All DOFs mode")
        # mode for all joints being controlled at once
        beforeDirectionVector = copy.deepcopy(joystickValues)
        #print(beforeDirectionVector)
        directionVector = [0,0,0,0,0,0] #beforeDirectionVector
        index = -1
        for thing in beforeDirectionVector:
            index += 1
            if abs(thing) > 0.05: # sensitivity "gap", to avoid random movements
                directionVector[index] = thing
        #print(directionVector)
    elif modeOfMovement == 1:
        print("One DOF mode")
        # mode for only one joint at once rotation
        # determine direction
        directionVector = [0,0,0,0,0,0]
        storedVal = 0
        storedInd = 0
        ind = -1
        absJoystickValues = np.absolute(np.matrix(joystickValues))
        storedInd = np.argmax(absJoystickValues)
        storedVal = joystickValues[storedInd]
        # introduce some "sensitivity gap" to avoid random movement
        if abs(storedVal) > 0.05:
            # if storedInd == 3 and modeOfOperation == 3:
            #     if savedJointAngles[5] > 0:
            #         directionVector[storedInd] = storedVal
            #     else:on
            #         directionVector[storedInd] = -storedVal
            # else:
            directionVector[storedInd] = storedVal
        #print(directionVector)
        
    # needed specifically to make thigs coincide with our arm
    for i in range( len(directionVector) ):
        directionVector[i] = -directionVector[i]
    #xyz > yxz > zxy
    # swap x with y
    tempval = copy.deepcopy( directionVector[0] )
    directionVector[0] = copy.deepcopy( directionVector[1] )
    directionVector[1] = tempval
    # in new translation swap z and y
    tempval2 = copy.deepcopy( directionVector[2] )
    directionVector[2] = copy.deepcopy( directionVector[0] )
    directionVector[0] = tempval2
    # swap the z direction
    directionVector[0] = -directionVector[0]
    # rotations swaps. Remember for positionalIK mode it's rotations above yxz order
    directionVector[3] = -directionVector[3]
    directionVector[4] = directionVector[4]
    directionVector[5] = -directionVector[5]
    return directionVector


def updateGripperAngle(localSavedGripperAngle): # updates the gripper condition (open/closed)
    buttons = getJoystickButtons()
    global limitFlag

    if limitFlag == False:
        openedLimit = 100000000
        closedLimit = -100000000
    else:
        openedLimit = 21000
        closedLimit = 0

    # servo moves in the range 0 -1023
    updatedGripperAngle = localSavedGripperAngle
    step = 200
    if buttons[22] == 1:
        if updatedGripperAngle+step <= openedLimit:
            updatedGripperAngle += step
        else:
            print("Gripper completely open")
    elif buttons[25] == 1:
        if updatedGripperAngle-step >= closedLimit:
            updatedGripperAngle -= step
        else:
            print("Gripper completely closed")

    return updatedGripperAngle
    

def visualizeArm(jointAngles):
    copyJointAngles = copy.deepcopy(jointAngles)
    #print("Values on visualization: {}".format(copyJointAngles))
    global robot
    global initAngles
    
    for i in range(len(copyJointAngles)):
        copyJointAngles[i] += initAngles[i]
    robot.SetActiveDOFValues(copyJointAngles)
    #print("Initial angles: {}".format(initAngles))


def sendMessage(message):
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
    

def makeDHTable(jointAngles): # making a dh table for kinematics, specific for the exact robot
    #print("Current joint angles: {}".format(jointAngles))
    #print(jointAngles)
    
	# DH Table with entries in the format: [a, alpha, d, theta]
    # First links are first entries
    DHTable = [ [0, math.pi/2, 5.5, jointAngles[0]],
                [36, 0, 0, jointAngles[1]],
                [0, math.pi/2, 0, jointAngles[2]],
                [0, -math.pi/2, 35.683, jointAngles[3]],
                [0, math.pi/2, 0, jointAngles[4]],
                [0, 0, 18, jointAngles[5]] ]
    return DHTable
'''
    ed
    DHTable = [ [0, math.pi/2, 6.162, jointAngles[0]],
                [36, 0, 0, jointAngles[1]],
                [27.94, 0, 0, jointAngles[2]],
                [23.682, math.pi/2, 0, jointAngles[3]],
                [0, math.pi/2, 0, jointAngles[4]],
                [0, 0, 10, jointAngles[5]] ]
                #end effector length not determined yet, 10 can change
'''

def updateAngles(DHTable, updatedDHTable, joystickDirection): # update angles during movement
    global k
    global modeOfOperation
    global qlim
    global maxRot

    if modeOfOperation == 1:
        #Manual mode
        uq1 = DHTable[0][3] + 0.002 * 2*math.pi * joystickDirection[1]#sr
        uq2 = DHTable[1][3] + 0.0006 * 2*math.pi * joystickDirection[2]#sp
        uq3 = DHTable[2][3] + 0.001 * 2*math.pi * joystickDirection[0]#eb
        uq4 = DHTable[3][3] + 0.005 * 2*math.pi * joystickDirection[4]#w3
        uq5 = DHTable[4][3] + 0.005 * 2*math.pi * joystickDirection[3]#w1
        uq6 = DHTable[5][3] + 0.005 * 2*math.pi * joystickDirection[5]#w2
    elif modeOfOperation == 2:
        #Positional IK mode
        uq1 = updatedDHTable[0][3]
        uq2 = updatedDHTable[1][3]
        uq3 = updatedDHTable[2][3]
        uq4 = DHTable[3][3] + updatedDHTable[3][3]
        uq5 = DHTable[4][3] + updatedDHTable[4][3]
        uq6 = DHTable[5][3] + updatedDHTable[5][3]
    elif modeOfOperation == 3:
        #Full IK mode
        uq1 = updatedDHTable[0][3]
        uq2 = updatedDHTable[1][3]
        uq3 = updatedDHTable[2][3]
        uq4 = updatedDHTable[3][3]
        uq5 = updatedDHTable[4][3]
        uq6 = updatedDHTable[5][3]
    uq = [uq1, uq2, uq3, uq4, uq5, uq6]
    update = 1
    for i in range(6):
        if(uq[i] <= qlim[i][0] or uq[i] >= qlim[i][1] or abs(uq[i] - DHTable[i][3]) > maxRot):
            update = 0
            break
    if(update == 0):
        for i in range(6):
            uq[i] = DHTable[i][3]       #Do not change angle if it exceeds the limits.


    return uq


def manual(): # manual mode of operating the arm
    # get the direction value to move in
    joystickDirection = getJoystickDirection()
    #print("Current joystick direction:")
    #print(joystickDirection)
    # get the current joint angles of the arm
    global savedJointAngles
    global savedGripperAngle
    jointAngles = copy.deepcopy(savedJointAngles)

    DHTable = makeDHTable(jointAngles)
    # do forward kinematics
    DHTableCopy = copy.deepcopy(DHTable)
    #homTransMatrix = forwardKinematics(DHTableCopy)
    #print("Current position: " )
    #print([ homTransMatrix[0][3], homTransMatrix[1][3], homTransMatrix[2][3] ])
    visualizeArm(jointAngles)
    
    uq = updateAngles(DHTable, 0, joystickDirection)

    # update gripper value
    gripperAngleNew = updateGripperAngle(savedGripperAngle)
    try:
        jointAngles = copy.deepcopy( uq )
        #print("Updated joint angles: {}".format(jointAngles))
        #print(jointAngles)
        savedJointAngles = copy.deepcopy(jointAngles)
        savedGripperAngle = gripperAngleNew
        
        # MOVE THE ARM TO THE NEW PLACE!!!!!!!!!!
        sendAngles = [ savedJointAngles[0], savedJointAngles[1], savedJointAngles[2], savedJointAngles[3], savedJointAngles[4], savedJointAngles[5], savedGripperAngle ]
        sendAngleValues(sendAngles)
        
        visualizeArm(savedJointAngles)
        print("Joint angles and gripper: \n Shoulder Rotation(q1): {} \n Shoulder Pitch(q2): {} \n Elbow(q3): {} \n W1(q4): {} \n W2(q5): {} \n W3(q6): {} \n Gripper value: {} \n".format( savedJointAngles[0]*180/math.pi, savedJointAngles[1]*180/math.pi, savedJointAngles[2]*180/math.pi, savedJointAngles[3]*180/math.pi, savedJointAngles[4]*180/math.pi, savedJointAngles[5]*180/math.pi, savedGripperAngle ) )
        #print( np.array(savedJointAngles) * 180/math.pi )
    except:
        print("Exception encountered")
        # TODO: fix undefined q1, etc here
        jointAngles = copy.deepcopy( [q1,q2,=q3,q4,q5,q6] )
        #print("Updated joint angles: {}".format(jointAngles)) 
        savedJointAngles = copy.deepcopy(jointAngles)
        savedGripperAngle = savedGripperAngle
        
        # MOVE THE ARM TO THE NEW PLACE!!!!!!!!!!
        sendAngles = [ savedJointAngles[0], savedJointAngles[1], savedJointAngles[2], savedJointAngles[3], savedJointAngles[4], savedJointAngles[5], savedGripperAngle ]
        sendAngleValues(sendAngles)
        
        visualizeArm(savedJointAngles)
        print("Joint angles and gripper: \n Shoulder Rotation(q1): {} \n Shoulder Pitch(q2): {} \n Elbow(q3): {} \n W1(q4): {} \n W2(q5): {} \n W3(q6): {} \n Gripper value: {} \n".format( savedJointAngles[0]*180/math.pi, savedJointAngles[1]*180/math.pi, savedJointAngles[2]*180/math.pi, savedJointAngles[3]*180/math.pi, savedJointAngles[4]*180/math.pi, savedJointAngles[5]*180/math.pi, savedGripperAngle ) )
        #print( np.array(savedJointAngles) * 180/math.pi )


def positionalIK(): # perform inverse kinematics, move the arm to the new position
    # get the direction value to move in
    joystickDirection = getJoystickDirection()
    #print("Current joystick direction:")
    #print(joystickDirection)
    # get the current joint angles of  the arm
    global savedJointAngles
    global savedGripperAngle
    jointAngles = copy.deepcopy(savedJointAngles)

    # DH Table with entries in the format: [a, alpha, d, theta]
    # First links are first entries
    DHTable = makeDHTable(jointAngles)

    # do forward kinematics
    DHTableCopy = copy.deepcopy(DHTable)
    homTransMatrix = forwardKinematics(DHTableCopy)
    #print("Current position: " )
    #print([ homTransMatrix[0][3], homTransMatrix[1][3], homTransMatrix[2][3] ])
    visualizeArm(jointAngles)

    # update homogenous transformation matrix based on joystick input
    translationVector = joystickDirection[:3]
    rotationVector = joystickDirection[3:]
    
    copyHomTransMatrix = copy.deepcopy(homTransMatrix)
    DHTableCopy2 = copy.deepcopy(DHTable)
    updatedHomTransMatrix = updateHomTransMatrixPositional(copyHomTransMatrix, DHTableCopy2, translationVector, rotationVector)
    #print("Updated position: ")
    #print([ updatedHomTransMatrix[0][3], updatedHomTransMatrix[1][3], updatedHomTransMatrix[2][3] ])

    # solve IK based on the new homTransMatrix
    DHTableCopy3 = copy.deepcopy(DHTable)
    copyUpdatedHomTransMatrix = copy.deepcopy(updatedHomTransMatrix)
    updatedDHTable = inverseKinematicsPositional(DHTableCopy3, copyUpdatedHomTransMatrix, rotationVector)

    uq = updateAngles(DHTable, updatedDHTable, joystickDirection)
    # update gripper value
    gripperAngleNew = updateGripperAngle(savedGripperAngle)
    try:
        jointAngles = copy.deepcopy( uq )
        #print("Updated joint angles: {}".format(jointAngles))
        #print(jointAngles)

        savedGripperAngle = gripperAngleNew
        
        # MOVE THE ARM TO THE NEW PLACE!!!!!!!!!!
        sendAngles = [ savedJointAngles[0], savedJointAngles[1], savedJointAngles[2], savedJointAngles[3], savedJointAngles[4], savedJointAngles[5], savedGripperAngle ]
        sendAngleValues(sendAngles)
        
        savedJointAngles = copy.deepcopy(jointAngles)
        visualizeArm(savedJointAngles)
        print("Joint angles and gripper: \n Shoulder Rotation(q1): {} \n Shoulder Pitch(q2): {} \n Elbow(q3): {} \n W1(q4): {} \n W2(q5): {} \n W3(q6): {} \n Gripper value: {} \n".format( savedJointAngles[0]*180/math.pi, savedJointAngles[1]*180/math.pi, savedJointAngles[2]*180/math.pi, savedJointAngles[3]*180/math.pi, savedJointAngles[4]*180/math.pi, savedJointAngles[5]*180/math.pi, savedGripperAngle ) )
        #print( np.array(savedJointAngles) * 180/math.pi )
    except:
        print("Exception encountered")
        jointAngles = copy.deepcopy( [q1,q2,q3,q4,q5,q6] )
        #print("Updated joint angles: {}".format(jointAngles))

        savedGripperAngle = savedGripperAngle
        
        # MOVE THE ARM TO THE NEW PLACE!!!!!!!!!!
        sendAngles = [ savedJointAngles[0], savedJointAngles[1], savedJointAngles[2], savedJointAngles[3], savedJointAngles[4], savedJointAngles[5], savedGripperAngle ]
        sendAngleValues(sendAngles)
        
        savedJointAngles = copy.deepcopy(jointAngles)
        visualizeArm(savedJointAngles)
        print("Joint angles and gripper: \n Shoulder Rotation(q1): {} \n Shoulder Pitch(q2): {} \n Elbow(q3): {} \n W1(q4): {} \n W2(q5): {} \n W3(q6): {} \n Gripper value: {} \n".format( savedJointAngles[0]*180/math.pi, savedJointAngles[1]*180/math.pi, savedJointAngles[2]*180/math.pi, savedJointAngles[3]*180/math.pi, savedJointAngles[4]*180/math.pi, savedJointAngles[5]*180/math.pi, savedGripperAngle ) )
        #print( np.array(savedJointAngles) * 180/math.pi )
    

def fullIK():
    # get the direction value to move in
    joystickDirection = getJoystickDirection()
    #print("Current joystick direction: {}".format(joystickDirection))
    # get the current joint angles of the arm
    global savedJointAngles
    global savedGripperAngle          
    jointAngles = copy.deepcopy(savedJointAngles)

    DHTable = makeDHTable(jointAngles)

    #print("Current position: " )
    #print([ homTransMatrix[0][3], homTransMatrix[1][3], homTransMatrix[2][3] ])
    visualizeArm(jointAngles)

    # update homogenous transformation matrix based on joystick input
    translationVector = joystickDirection[:3]
    rotationVector = joystickDirection[3:]

    DHTableCopy = copy.deepcopy(DHTable)
    homTransMatrix = forwardKinematics(DHTableCopy)
    #print homTransMatrix
    copyHomTransMatrix = copy.deepcopy(homTransMatrix)
    updatedHomTransMatrix = updateHomTransMatrix(copyHomTransMatrix, DHTableCopy, translationVector, rotationVector)
    #print("Updated position: ")
    #print updatedHomTransMatrix

    # solve IK based on the new homTransMatrix
    DHTableCopy2 = copy.deepcopy(DHTable)
    #print "DHTable: "
    #print DHTableCopy2
    copyUpdatedHomTransMatrix = copy.deepcopy(updatedHomTransMatrix)
    updatedDHTable = inverseKinematics(DHTableCopy2, copyUpdatedHomTransMatrix)
    #print "DHTable updated: "
    #print updatedDHTable
    
    uq = updateAngles(DHTable, updatedDHTable, joystickDirection)
    # update gripper value
    gripperAngleNew = updateGripperAngle(savedGripperAngle)
    try:
        jointAngles = copy.deepcopy( uq )
        #print("Updated joint angles: {}".format(jointAngles))
        
        savedJointAngles = copy.deepcopy(jointAngles)

        savedGripperAngle = gripperAngleNew
        
        # MOVE THE ARM TO THE NEW PLACE!!!!!!!!!!
        sendAngles = [ savedJointAngles[0], savedJointAngles[1], savedJointAngles[2], savedJointAngles[3], savedJointAngles[4], savedJointAngles[5], savedGripperAngle ]
        sendAngleValues(sendAngles)
        
        visualizeArm(savedJointAngles)
        print("Joint angles and gripper: \n Shoulder Rotation(q1): {} \n Shoulder Pitch(q2): {} \n Elbow(q3): {} \n W1(q4): {} \n W2(q5): {} \n W3(q6): {} \n Gripper value: {} \n".format( savedJointAngles[0]*180/math.pi, savedJointAngles[1]*180/math.pi, savedJointAngles[2]*180/math.pi, savedJointAngles[3]*180/math.pi, savedJointAngles[4]*180/math.pi, savedJointAngles[5]*180/math.pi, savedGripperAngle ) )
        #print( np.array(savedJointAngles) * 180/math.pi )
    except:
        print("Exception encountered")
        jointAngles = copy.deepcopy( [q1,q2,q3,q4,q5,q6] )
        #print("Updated joint angles: {}".format(jointAngles))
        savedJointAngles = copy.deepcopy(jointAngles)

        savedGripperAngle = savedGripperAngle
        
        # MOVE THE ARM TO THE NEW PLACE!!!!!!!!!! or just do nothing?
        sendAngles = [ savedJointAngles[0], savedJointAngles[1], savedJointAngles[2], savedJointAngles[3], savedJointAngles[4], savedJointAngles[5], savedGripperAngle ]
        sendAngleValues(sendAngles)
        
        visualizeArm(savedJointAngles)
        print("Joint angles and gripper: \n Shoulder Rotation(q1): {} \n Shoulder Pitch(q2): {} \n Elbow(q3): {} \n W1(q4): {} \n W2(q5): {} \n W3(q6): {} \n Gripper value: {} \n".format( savedJointAngles[0]*180/math.pi, savedJointAngles[1]*180/math.pi, savedJointAngles[2]*180/math.pi, savedJointAngles[3]*180/math.pi, savedJointAngles[4]*180/math.pi, savedJointAngles[5]*180/math.pi, savedGripperAngle ) )
        #print( np.array(savedJointAngles) * 180/math.pi )


def directControl(): # move the arm according to the joystick position
    global THREAD_MODE_FLAG
    # get the direction value to move in
    joystickDirection = getJoystickDirection()
    global k
    sensitivity = k
    i = 0
    for elem in joystickDirection:
        modElem = elem * sensitivity
        if abs(modElem) > 1:
            if modElem > 1:
                modElem = 1.0
            elif modElem < -1:
                modElem = -1.0
        joystickDirection[i] = modElem
        i += 1
    # print("Current joystick direction:")
    # print(joystickDirection)
    # get the current joint angles of the arm
    global savedJointAngles
    jointAngles = copy.deepcopy(savedJointAngles)

    visualizeArm(jointAngles)

    buttons = getJoystickButtons()
    
    try:
        
        # MOVE THE ARM TO THE NEW PLACE!!!!!!!!!!
        if buttons[22] == 1:
            gripperSpeed = -255
        elif buttons[25] == 1:
            gripperSpeed = 255
        else:
            gripperSpeed = 0
        sendSpeeds = [ int(joystickDirection[1] * 255), int(joystickDirection[2] * 255), int(joystickDirection[0] * 255), int(-joystickDirection[4] * 255), int(joystickDirection[5] * 255), int(joystickDirection[3] * 255), gripperSpeed ]
        
        sendAngleValues(sendSpeeds)
        
        
        visualizeArm(savedJointAngles)
        print("Joint angles and gripper: \n Shoulder Rotation(q1): {} \n Shoulder Pitch(q2): {} \n Elbow(q3): {} \n W1(q4): {} \n W2(q5): {} \n W3(q6): {} \n Gripper value: {} \n".format( savedJointAngles[0]*180/math.pi, savedJointAngles[1]*180/math.pi, savedJointAngles[2]*180/math.pi, savedJointAngles[3]*180/math.pi, savedJointAngles[4]*180/math.pi, savedJointAngles[5]*180/math.pi, savedGripperAngle ) )
        #print( np.array(savedJointAngles) * 180/math.pi )
    except:
        print("Exception encountered")
        
        # MOVE THE ARM TO THE NEW PLACE!!!!!!!!!!
        sendSpeeds = [ 0, 0, 0, 0, 0, 0, 0 ]
        
        sendAngleValues(sendSpeeds)
             
        visualizeArm(savedJointAngles)
        print("Joint angles and gripper: \n Shoulder Rotation(q1): {} \n Shoulder Pitch(q2): {} \n Elbow(q3): {} \n W1(q4): {} \n W2(q5): {} \n W3(q6): {} \n Gripper value: {} \n".format( savedJointAngles[0]*180/math.pi, savedJointAngles[1]*180/math.pi, savedJointAngles[2]*180/math.pi, savedJointAngles[3]*180/math.pi, savedJointAngles[4]*180/math.pi, savedJointAngles[5]*180/math.pi, savedGripperAngle ) )
        #print( np.array(savedJointAngles) * 180/math.pi )

def updateOperationMode(): # update the mode of operation based on actions on the joystick
    global modeOfOperation
    global savedJointAngles

    buttons = getJoystickButtons()
    if buttons[28] == 1:
        modeOfOperation = 1
        print("Switched to manual mode")
        return modeOfOperation
    elif buttons[27] == 1:
        modeOfOperation = 2
        print("Switched to positional IK mode")
        return modeOfOperation
    elif buttons[26] == 1:
        modeOfOperation = 3
        print("Switched to full IK mode")
        return 
    elif buttons[6] == 1:
        modeOfOperation = 4
        print("Switched to Direct control mode")
        return modeOfOperation

def updateModeOfMovement(): # mode of movement - degrees of freedom involved 
    global modeOfMovement

    buttons = getJoystickButtons()
    if buttons[24] == 1:
        modeOfMovement = 0
        print("Switched to all DOFs mode")
        return modeOfMovement
    elif buttons[0] == 1:
        modeOfMovement = 1
        print("Switched to one DOF mode")
        return modeOfMovement


def updateSpeed(): # update on the speed based on joystick
    global k, t
    buttons = getJoystickButtons()

    if buttons[29] == 1:
        k *= 1.5
        t *= 1.5
        print("Speed increased")
    if buttons[30] == 1:
        k /= 1.5
        t /= 1.5
        print("Speed decreased")
    while buttons[30] or buttons[29]:
        buttons = getJoystickButtons()
        continue


def updateLimitMode(): # angle limit mode - limited or no limits
    global lim_q1_min, lim_q1_max, lim_q2_min, lim_q2_max, lim_q3_min, lim_q3_max, lim_q4_min, lim_q4_max, lim_q5_min, lim_q5_max, lim_q6_min, lim_q6_max
    global qlim
    global limitFlag

    buttons = getJoystickButtons()
    if buttons[7] == 1:
        qlim = np.array([[-1000000, 1000000], [-1000000, 1000000], [-1000000, 1000000], [-1000000, 1000000], [-1000000, 1000000], [-1000000, 1000000]]) * math.pi/180
        #print qlim
        limitFlag = False
        print("Switched to NO LIMITS to angles mode")
    elif buttons[21] == 1:
        qlim = np.array([[lim_q1_min, lim_q1_max], [lim_q2_min, lim_q2_max], [lim_q3_min, lim_q3_max], [lim_q4_min, lim_q4_max], [lim_q5_min, lim_q5_max], [lim_q6_min, lim_q6_max]]) * math.pi/180
        #print qlim
        limitFlag = True
        print("Switched to Limited angles mode")


def updateIKType(): # switch between ik axes - relative to the camera vs the tip of the arm
    global ikType

    if buttons[19] == 1:
        ikType = 0
        print("Switched to relative to the camera IK mode")
    elif buttons[20] == 1:
        ikType = 1
        print("Switched to relative to the tip IK mode")




def main():
    # TODO GET THE MODE OF OPERATION
    # modes: '1'-manual, '2'-positional IK (first 3 joints), '3'-full IK
    global modeOfOperation
    # ikTypes: '0'-relative to the camera, '1'-relative to the tip
    global ikType
    global storageFile
    # resetting the IK model to zero position upon request 
    global savedJointAngles
    global savedGripperAngle
    global limitFlag

    global qlim

    buttons = getJoystickButtons()
    # Move arm to the "Initial" pose
    if buttons[11] == 1:
        init_q2 = 1956.0/(2048*4)*360
        init_q3 = 1263.0/(2048*4)*360
        savedJointAngles = np.array([0.0000000001,init_q2,init_q3,0.0000000001,0.0000000001,0.0000000001]) * math.pi/180 # degrees
        print("Arm reset to the Initial pose")
    # Move arm to the "Forward" pose
    if buttons[17] == 1:
        savedJointAngles = np.array([0.0000000001,74.571,-39.836,0.0000000001,55.265,0.0000000001]) * math.pi/180 # degrees
        print("Arm reset to the Forward pose")
    # # Move arm to the "Left-Forward" pose
    # if buttons[19] == 1:
    #     savedJointAngles = np.array([19.847,65.963,-26.376,70.106,74.285,-36.814]) * math.pi/180 # degrees
    #     print("Arm reset to the Left-Forward pose")
    # # Move arm to the "Right-Forward" pose
    # if buttons[20] == 1:
    #     savedJointAngles = np.array([-19.847,65.963,-26.376,-70.106,74.285,36.814]) * math.pi/180 # degrees
    #     print("Arm reset to the Right-Forward pose")
    # Move arm to the "Down" pose
    if buttons[18] == 1:
        savedJointAngles = np.array([0.0000000001,60.960,-7.230,0.0000000001,-61.231,0.0000000001]) * math.pi/180 # degrees
        print("Arm reset to the Down pose")

    updateOperationMode()
    updateModeOfMovement()
    updateSpeed()
    updateLimitMode()
    updateIKType()
    if modeOfOperation == 1:
        print("Manual mode")
        if limitFlag == True:
            print(" Limited angles mode")
            #print qlim
        elif limitFlag == False:
            print("NO LIMITS to angles mode")
            #print qlim
        manual()
    elif modeOfOperation == 2:
        print("Positional IK mode")
        if limitFlag == True:
            print("Limited angles mode")
            #print qlim
        elif limitFlag == False:
            print("NO LIMITS to angles mode")
            #print qlim
        positionalIK()
    elif modeOfOperation == 3:
        print("Full IK mode")
        if limitFlag == True:
            print("Limited angles mode")
            #print qlim
        elif limitFlag == False:
            print("NO LIMITS to angles mode")
            #print qlim
        if ikType == 0:
            print("IK relative to the camera")
        elif ikType == 1:
            print("IK relative to the tip")
        fullIK()
    elif modeOfOperation == 4:
        print("Direct control mode")
        directControl()

    # saving the current arm status just in case
    storageFile = open('src/ArmControl/savedJointAngles.txt', 'w')
    storageFile.write( str(savedJointAngles) )
    storageFile.close()
    storageFile = open('src/ArmControl/savedGripperAngle.txt', 'w')
    storageFile.write( str(savedGripperAngle) )
    storageFile.close()


        


if __name__ == "__main__":
    global savedJointAngles
    global savedGripperAngle
    global modeOfOperation
    global storageFile
    global modeOfMovement # either motion in every DOF at once or only one DOF at once, "0" - all DOFs, "1" - one DOF
    global k, t # velocity coefficients for translational and rotational motions
    global qlim
    global maxRot # determines max rotation by a joint per turn
    # declare limit angle variables as global to easily change between limited and limitless modes
    global lim_q1_min, lim_q1_max, lim_q2_min, lim_q2_max, lim_q3_min, lim_q3_max, lim_q4_min, lim_q4_max, lim_q5_min, lim_q5_max, lim_q6_min, lim_q6_max
    global limitFlag
    global ikType
    # initializing ROS node
    global pub
    pub = rospy.Publisher('arm', String, queue_size=10)
    rospy.init_node('arm_talker', anonymous=True)
    global rate
    rate = rospy.Rate(100) # in Hz

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
    modeOfMovement = 1 # One DOF mode by default
    modeOfOperation = 2 # positional IK mode by default
    ikType = 0 # by default move in the camera reference frame

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
        #print( angles )
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

    qlim = np.array([[lim_q1_min, lim_q1_max], [lim_q2_min, lim_q2_max], [lim_q3_min, lim_q3_max], [lim_q4_min, lim_q4_max], [lim_q5_min, lim_q5_max], [lim_q6_min, lim_q6_max]]) * math.pi/180
    #qlim = np.array([[-18000, 18000], [-18000, 18000], [-18000, 18000], [-18000, 18000], [-18000, 18000], [-18000, 18000]]) * math.pi/180
    #print qlim
    setupVisualEnv()
    initializeJoystick()
    
    while True:

        # frequency of the model loop in Hz
        frequency = 50
        timeDelay =  1.0/frequency
        #print(timeDelay)
        if modeOfOperation == 4: # need high frequency reliably for the directControl mode, so set a separate frequency specifically for it
            time.sleep(0.02)
        else:
            time.sleep(timeDelay)

        turnedOn = True
        axes = getJoystickAxes()
        buttons = getJoystickButtons()
        #print axes
        #print buttons
        # absAxesSum = 0
        # for axis in axes:
        #     absAxesSum += abs(axis)
        # #print absAxesSum
        # # if one of the buttons pressed or one of the axes moved, move on!
        # if (absAxesSum > 0.1 or 1 in buttons):
        #     turnedOn = True
        #print turnedOn
        # TODO
        breakTrigger = False #GET THE BREAK TRIGGER FROM SOMEWHERE
        if turnedOn:
            if breakTrigger:
                print("Break triggered")
                break
            else:
                main()
                #continue
        else:
            continue

    
    print("Shut the operations down")
    