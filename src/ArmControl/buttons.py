#! /usr/bin/env python

import pygame
import time


def initializeJoystick():
    pygame.init()
    global joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print('Initialized joystick: %s' % joystick.get_name())


def getJoystickButtons():
    pygame.event.pump()
    
    buttons = []
    for i in range(0, joystick.get_numbuttons()):
        button = joystick.get_button(i)
        buttons.append(button)
    #print(buttons)
    return buttons



if __name__ == "__main__":
	initializeJoystick()

	while True:
		buttons = getJoystickButtons()
		print(buttons)
		time.sleep(1)