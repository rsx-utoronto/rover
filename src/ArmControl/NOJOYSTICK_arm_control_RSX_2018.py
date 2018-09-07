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
BLACK = (0, 0, 0)

pygame.init()
size = [200, 200]
screen = pygame.display.set_mode(size)
pygame.mouse.set_visible(0)
screen.fill(BLACK)


def sendAngleValues(qVect, start = 0):
    global limitFlag
    global modeOfOperation
    # generate messages from qVect here q1String etc correspond to order in message, not exactly in qVect
    q1String = str( qVect[0] )
    q2String = str( qVect[1] )
    q3String = str( qVect[2] )
    q4String = str( qVect[3] )
    q5String = str( qVect[4] )
    q6String = str( qVect[5] )
    q7String = str( qVect[6] ) # gripper
    
    command = 'm'
    message = command+"%20"+q1String+"%20"+q2String+"%20"+q3String+"%20"+q4String+"%20"+q5String+"%20"+q6String+"%20"+q7String
    print message
    sendMessage(message)


def sendMessage(message):
    global conn
    global connFlag

    if connFlag == 1:
        conn.request("PUT","/arm/"+message+"/")
        conn.close()


sendSpeeds = [0, 0, 0, 0, 0, 0, 0]


def directControl():
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                sendSpeeds[0] += 30
            if event.key == pygame.K_w:
                sendSpeeds[1] += 255
            if event.key == pygame.K_e:
                sendSpeeds[2] += 255
            if event.key == pygame.K_r:
                sendSpeeds[3] += 35
            if event.key == pygame.K_t:
                sendSpeeds[4] += 35
            if event.key == pygame.K_y:
                sendSpeeds[5] += 35
            if event.key == pygame.K_u:
                sendSpeeds[6] += 255
            if event.key == pygame.K_a:
                sendSpeeds[0] -= 30
            if event.key == pygame.K_s:
                sendSpeeds[1] -= 255
            if event.key == pygame.K_d:
                sendSpeeds[2] -= 255
            if event.key == pygame.K_f:
                sendSpeeds[3] -= 35
            if event.key == pygame.K_g:
                sendSpeeds[4] -= 35
            if event.key == pygame.K_h:
                sendSpeeds[5] -= 35
            if event.key == pygame.K_j:
                sendSpeeds[6] -= 255
            if event.key == pygame.K_SPACE:
                for e in sendSpeeds:
                    e = 0
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_q:
                sendSpeeds[0] -= 30
            if event.key == pygame.K_w:
                sendSpeeds[1] -= 255
            if event.key == pygame.K_e:
                sendSpeeds[2] -= 255
            if event.key == pygame.K_r:
                sendSpeeds[3] -= 35
            if event.key == pygame.K_t:
                sendSpeeds[4] -= 35
            if event.key == pygame.K_y:
                sendSpeeds[5] -= 35
            if event.key == pygame.K_u:
                sendSpeeds[6] -= 255
            if event.key == pygame.K_a:
                sendSpeeds[0] += 30
            if event.key == pygame.K_s:
                sendSpeeds[1] += 255
            if event.key == pygame.K_d:
                sendSpeeds[2] += 255
            if event.key == pygame.K_f:
                sendSpeeds[3] += 35
            if event.key == pygame.K_g:
                sendSpeeds[4] += 35
            if event.key == pygame.K_h:
                sendSpeeds[5] += 35
            if event.key == pygame.K_j:
                sendSpeeds[6] += 255
    sendAngleValues(sendSpeeds)


def main():
    # TODO GET THE MODE OF OPERATION
    # modes: '1'-manual, '2'-positional IK (first 3 joints), '3'-full IK
    global modeOfOperation
    # ikTypes: '0'-relative to the camera, '1'-relative to the tip
    global ikType
    global storageFile
    # resetting the IK model to zero position upon request 
    global savedJointAngles
    global limitFlag

    global qlim

    modeOfOperation = 4

    directControl()
        


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

    # do networking if connFlag == 1, don't otherwise
    global connFlag
    connFlag = 1;

    maxRot = 2*math.pi*10000/360 
    k = 0.6
    t = 0.03
    modeOfMovement = 1 # One DOF mode by default
    modeOfOperation = 2 # positional IK mode by default
    ikType = 0 # by default move in the camera reference frame

    serverIP = '192.168.0.3'
    serverHttpPort = '8080'
    global conn
    conn = httplib.HTTPConnection(serverIP+":"+serverHttpPort)

    init_q2 = 1956.0/(2048*4)*360
    init_q3 = 1263.0/(2048*4)*360
    savedJointAngles = np.array([0,init_q2,init_q3,0,0,0]) * math.pi/180 # degrees

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
    savedGripperAngle = 0
    
    while True:
        connFlag = 1;
        # frequency of the loop in Hz
        frequency = 50
        timeDelay =  1.0/frequency
        time.sleep(0.02)

        main()    
