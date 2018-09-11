#! /usr/bin/env python
# imports
import time
import pygame
import math
import httplib
import sys
import rospy
from std_msgs.msg import String

BLUE = (0, 0, 255)

pygame.init()
size = [300, 200]
screen = pygame.display.set_mode(size)
pygame.display.set_caption("DRILL")
pygame.mouse.set_visible(0)
screen.fill(BLUE)


def send_movement(speeds):
    message = 'm'
    for speed in speeds:
        message += " " + str(int(speed))
    print message
    sendMessage(message)


def sendMessage(message):
    global pub
    if not rospy.is_shutdown():
        pub.publish(message)
        rate.sleep()
    #global conn
    #conn.request("PUT", "/arm/" + message + "/")
    #conn.close()


sendSpeeds = [0, 0, 0, 0, 0, 0, 0]
drill_speed = 50
platform_speed = 50


def process_keyboard():
    global sendSpeeds
    global drill_speed
    global platform_speed
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                sendSpeeds[3] += platform_speed
            if event.key == pygame.K_DOWN:
                sendSpeeds[3] -= platform_speed
            if event.key == pygame.K_RIGHT:
                sendSpeeds[1] -= drill_speed
            if event.key == pygame.K_LEFT:
                sendSpeeds[1] += drill_speed
            if event.key == pygame.K_1:
                drill_speed = 25
            if event.key == pygame.K_2:
                drill_speed = 50
            if event.key == pygame.K_3:
                drill_speed = 75
            if event.key == pygame.K_4:
                drill_speed = 100
            if event.key == pygame.K_5:
                drill_speed = 125
            if event.key == pygame.K_6:
                drill_speed = 150
            if event.key == pygame.K_7:
                drill_speed = 175
            if event.key == pygame.K_8:
                drill_speed = 200
            if event.key == pygame.K_9:
                drill_speed = 225
            if event.key == pygame.K_0:
                drill_speed = 255
            if event.key == pygame.K_q:
                platform_speed = 10
            if event.key == pygame.K_w:
                platform_speed = 20
            if event.key == pygame.K_e:
                platform_speed = 30
            if event.key == pygame.K_r:
                platform_speed = 40
            if event.key == pygame.K_t:
                platform_speed = 60
            if event.key == pygame.K_y:
                platform_speed = 90
            if event.key == pygame.K_u:
                platform_speed = 130
            if event.key == pygame.K_i:
                platform_speed = 180
            if event.key == pygame.K_o:
                platform_speed = 225
            if event.key == pygame.K_p:
                platform_speed = 255
            if event.key == pygame.K_SPACE:
                sendSpeeds = [0, 0, 0, 0, 0, 0, 0]
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_UP:
                sendSpeeds[3] -= platform_speed
            if event.key == pygame.K_DOWN:
                sendSpeeds[3] += platform_speed
            if event.key == pygame.K_RIGHT:
                sendSpeeds[1] += drill_speed
            if event.key == pygame.K_LEFT:
                sendSpeeds[1] -= drill_speed
    send_movement(sendSpeeds)


def main():
    # initializing ROS node
    global pub
    pub = rospy.Publisher('arm', String, queue_size=10)
    rospy.init_node('arm_talker', anonymous=True)
    global rate
    rate = rospy.Rate(100) # in Hz


    #global conn
    serverIP = '192.168.0.3'
    serverHttpPort = '8080'
    #conn = httplib.HTTPConnection(serverIP+":"+serverHttpPort)
    while True:
        process_keyboard()
        time.sleep(0.02)


if __name__ == "__main__":
    main()
