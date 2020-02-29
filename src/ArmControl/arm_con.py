#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray

import pygame

def initialize_joystick():
	pygame.init()
	global joystick
	joystick = pygame.joystick.Joystick(0)
	joystick.init()
	rospy.loginfo('Initialized joystick: %s' % joystick.get_name())

def joystick_axes():
    out = [0,0,0,0,0,0]
    for i in range(0, joystick.get_numaxes()):
        out[i] = joystick.get_axis(i)
    return out

def joystick_buttons():
	buttons = []
	for i in range(0, joystick.get_numbuttons()):
		button = joystick.get_button(i)
		buttons.append(button)
	return buttons

def axes_to_pwm(buttons, axes):
	out = [0,0,0,0,0,0,0]
	out[0] = buttons[22] * 255 - buttons[25] * 255
	out[1] = axes[0] * 255
	out[2] = axes[1] * 255
	out[3] = axes[2] * 255
	out[4] = axes[3] * 255
	out[5] = axes[4] * 255
	out[6] = axes[5] * 255
	return out

if __name__ == "__main__":
	initialize_joystick()
	arm_direct_pwm_pub = rospy.Publisher('rover/arm_direct_pwm', Int16MultiArray, queue_size=10)
	rospy.init_node('arm_con', anonymous=True)

	rate = rospy.Rate(100) # in Hz	
	while not rospy.is_shutdown():
		pygame.event.pump() # update the joystick

		axes = joystick_axes()
		buttons = joystick_buttons()

		arm_direct_pwm_msg = Int16MultiArray(data = axes_to_pwm(buttons, axes))
		arm_direct_pwm_pub.publish(arm_direct_pwm_msg)
		
		rate.sleep()
