#! /usr/bin/env python

import pygame
import serial
import sys

ser = serial.Serial('/dev/ttyACM0', 9600)
 
# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
 
 
# def draw_stick_figure(screen, x, y):
#     # Head
#     pygame.draw.ellipse(screen, BLACK, [1 + x, y, 10, 10], 0)
 
#     # Legs
#     pygame.draw.line(screen, BLACK, [5 + x, 17 + y], [10 + x, 27 + y], 2)
#     pygame.draw.line(screen, BLACK, [5 + x, 17 + y], [x, 27 + y], 2)
 
#     # Body
#     pygame.draw.line(screen, RED, [5 + x, 17 + y], [5 + x, 7 + y], 2)
 
#     # Arms
#     pygame.draw.line(screen, RED, [5 + x, 7 + y], [9 + x, 17 + y], 2)
#     pygame.draw.line(screen, RED, [5 + x, 7 + y], [1 + x, 17 + y], 2)
 
# Setup
pygame.init()

# Set the width and height of the screen [width,height]
size = [700, 500]
screen = pygame.display.set_mode(size)
 
pygame.display.set_caption("My Game")
 
# Loop until the user clicks the close button.
#done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()
 
# Hide the mouse cursor
pygame.mouse.set_visible(0)
 
# Speed in pixels per frame
#x_speed = 0
#y_speed = 0
 
# Current position
#x_coord = 10
#y_coord = 10
 
# -------- Main Program Loop -----------
# list of keys in order: Q, E, W, S, A, D, R, F, U, J, I, K
list_of_key_numbers_unpressed = [11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33]
done = False

while not done:
    print(1)
    # OUTPUT TO SERIAL
    ser.write(chr(1))

    # --- Event Processing
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key==pygame.K_q or event.key==pygame.K_e or event.key==pygame.K_w or event.key==pygame.K_s or event.key==pygame.K_a or event.key==pygame.K_d or event.key==pygame.K_r or event.key==pygame.K_f or event.key==pygame.K_u or event.key==pygame.K_j or event.key==pygame.K_i or event.key==pygame.K_k or event.key==pygame.K_SPACE or event.key==pygame.K_ESCAPE:
                keys_pressed = pygame.key.get_pressed()
                #print( keys_pressed )
                space = keys_pressed[32]
                #print(space)
                #if space == 1:
                #	done = True
                #	continue

                q = keys_pressed[113]
                #print(q)
                e = keys_pressed[101]
                #print(e)
                w = keys_pressed[119]
                #print(w)
                s = keys_pressed[115]
                #print(s)
                a = keys_pressed[97]
                #print(a)
                d = keys_pressed[100]
                #print(d)
                r = keys_pressed[114]
                #print(r)
                f = keys_pressed[102]
                #print(f)
                u = keys_pressed[117]
                #print(u)
                j = keys_pressed[106]
                #print(j)
                i = keys_pressed[105]
                #print(i)
                k = keys_pressed[107]
                #print(k)
                pressed = [q, e, w, s, a, d, r, f, u, j, i, k]
                #print( ' Q  E  W  S  A  D  R  F  U  J  I  K ' )
                #print( pressed )

                if event.key == pygame.K_q:
                    output = 10
                elif event.key == pygame.K_e:
                    output = 12
                elif event.key == pygame.K_w:
                    output = 14
                elif event.key == pygame.K_s:
                    output = 16
                elif event.key == pygame.K_a:
                    output = 18
                elif event.key == pygame.K_d:
                    output = 20
                elif event.key == pygame.K_r:
                    output = 22
                elif event.key == pygame.K_f:
                    output = 24
                elif event.key == pygame.K_u:
                    output = 26
                elif event.key == pygame.K_j:
                    output = 28
                elif event.key == pygame.K_i:
                    output = 30
                elif event.key == pygame.K_k:
                    output = 32
                elif event.key == pygame.K_SPACE:
                    output = 2
                elif event.key == pygame.K_ESCAPE:
                    output = 2
                    done = True


                #output = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                #for i in range(len(output)):
                #	output[i] = list_of_key_numbers_unpressed[i] - pressed[i]
                #print('Output: ')
                print( output )
                # OUTPUT TO SERIAL
                ser.write(chr(output))

        if event.type == pygame.KEYUP:
            if event.key==pygame.K_q or event.key==pygame.K_e or event.key==pygame.K_w or event.key==pygame.K_s or event.key==pygame.K_a or event.key==pygame.K_d or event.key==pygame.K_r or event.key==pygame.K_f or event.key==pygame.K_u or event.key==pygame.K_j or event.key==pygame.K_i or event.key==pygame.K_k:
                keys_pressed = pygame.key.get_pressed()
                #print( keys_pressed )
                space = keys_pressed[32]
                #print(space)
                #if space == 1:
                #    done = True
                #    continue

                q = keys_pressed[113]
                #print(q)
                e = keys_pressed[101]
                #print(e)
                w = keys_pressed[119]
                #print(w)
                s = keys_pressed[115]
                #print(s)
                a = keys_pressed[97]
                #print(a)
                d = keys_pressed[100]
                #print(d)
                r = keys_pressed[114]
                #print(r)
                f = keys_pressed[102]
                #print(f)
                u = keys_pressed[117]
                #print(u)
                j = keys_pressed[106]
                #print(j)
                i = keys_pressed[105]
                #print(i)
                k = keys_pressed[107]
                #print(k)
                pressed = [q, e, w, s, a, d, r, f, u, j, i, k]
                #print( ' Q  E  W  S  A  D  R  F  U  J  I  K ' )
                #print( pressed )


                if event.key == pygame.K_q:
                    output = 11
                elif event.key == pygame.K_e:
                    output = 13
                elif event.key == pygame.K_w:
                    output = 15
                elif event.key == pygame.K_s:
                    output = 17
                elif event.key == pygame.K_a:
                    output = 19
                elif event.key == pygame.K_d:
                    output = 21
                elif event.key == pygame.K_r:
                    output = 23
                elif event.key == pygame.K_f:
                    output = 25
                elif event.key == pygame.K_u:
                    output = 27
                elif event.key == pygame.K_j:
                    output = 29
                elif event.key == pygame.K_i:
                    output = 31
                elif event.key == pygame.K_k:
                    output = 33

                #output = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                #for i in range(len(output)):
                #    output[i] = list_of_key_numbers_unpressed[i] - pressed[i]
                #print('Output: ')
                print( output )
                # OUTPUT TO SERIAL
                ser.write(chr(output))

 
    # --- Drawing Code
 
    # First, clear the screen to WHITE. Don't put other drawing commands
    # above this, or they will be erased with this command.
    screen.fill(WHITE)
 
    #draw_stick_figure(screen, x_coord, y_coord)
 
 
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()
 
    # Limit frames per second
    time_delay = 0.03 # in seconds
    clock.tick( int(1/time_delay) )
 
# Close the window and quit.
ser.close()
pygame.quit()

