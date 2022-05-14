#!/usr/bin/python3
import RPi.GPIO as GPIO 
from time import sleep
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import getch   
import math
import numpy as np

class roverControl:
    def __init__(self):
        self.pub_twist_fake = rospy.Publisher("rover_twist_fake", Twist, queue_size=10)
        self.pub_twist = rospy.Publisher("rover_twist", Twist, queue_size=10)

        # For larger rover implementation
        self.lin_vel = 0.3
        self.ang_vel = 1
        self.ref_ang_vel = 1

        self.arrived = False

        # t = threading.Thread(target = motorListener, name = 'listener_thread')
        # t.start()
    
    def cleanup(self):
        GPIO.cleanup()

    def autoControlCallback(self, angle_from_rover):
        # target angle must always be given relative to the 0 degrees for the robot 
        twist_fake = Twist()
        twist_real = Twist()
        angle_from_rover = angle_from_rover.data
        if angle_from_rover >= 0:
            #Turn right
            if angle_from_rover < 85:
                self.ang_vel = self.ref_ang_vel * abs(math.cos(math.radians(abs(angle_from_rover))))
                self.lin_vel = 0.3 # cannot be used for mini rover

            #For large values just turn
            else:
                self.ang_vel = self.ref_ang_vel
                self.lin_vel = 0 #stop and turn
                #self.turnRight() # mini rover

        else:
            #Turn left
            if abs(angle_from_rover) < 85:
                self.lin_vel = 0.3
                angular_velocity = -self.ref_ang_vel * abs(math.cos(math.radians(abs(angle_from_rover))))
                #self.turnLeft() # mini rover 
            #For large angles just turn
            else:
                angular_velocity = -self.reg_ang_vel
                self.lin_vel = 0 #stop and turn
                #self.turnLeft() # mini rover 
        twist_real.linear.y = self.lin_vel
        twist_real.angular.z = self.ang_vel

        choice_linear = [1]
        choice_angular = [0]
        twist_fake.linear.y = np.random.choice(choice_linear)
        twist_fake.linear.z = np.random.choice(choice_angular)

        self.pub_twist_fake.publish(twist_fake)
        self.pub_twist.publish(twist_real)

        print("Linear velocity: " + str(self.lin_vel) + " Angular velocity: " + str(self.ang_vel))    
    
    def controlListener(self):
        rospy.init_node('rover_control_node')
        rospy.Subscriber('/target_angle', Float64, self.autoControlCallback)
        rospy.spin()   # see if I can change frequency to slower than inertial sense provision     

    def manualControl(self):
        rate = rospy.Rate(5)
        twist = Twist()
        while not rospy.is_shutdown():
            print("Key press?")
            press = 'q'
            press = input()
            if press == "w":
                self.lin_vel = 1
                self.ang_vel = 0
                press == "s"
            elif press == "s":
                self.lin_vel = -1
                self.ang_vel = 0
                press == "s"
            elif press == "a":
                self.lin_vel = 0
                self.ang_vel = -1
                press == "s"
            elif press == "d":
                self.lin_vel = 0
                self.ang_vel = 1
                press == "s"
            elif press == "q":
                self.lin_vel = 0
                self.ang_vel = 0
            twist.angular.z = self.ang_vel
            twist.linear.y = self.lin_vel

            self.pub_twist.publish(twist)
            rate.sleep()
        self.cleanup()    


# set up for testing not as an encoder and motor node right now    
if __name__ == '__main__':
    rover_control = roverControl()
    rospy.init_node('rover_control_node')
    while not rospy.is_shutdown():
        rover_control.controlListener()
    print("Control node is shutting down ...")