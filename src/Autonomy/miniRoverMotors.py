#!/usr/bin/python3
import RPi.GPIO as GPIO 
from time import sleep
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import getch   

class setMotor:
    def __init__(self, in1, in2, name):
        self.in1 = in1
        self.in2 = in2
        self.name = name 

    def setup(self):
        GPIO.setup(self.in1,GPIO.OUT)
        GPIO.setup(self.in2,GPIO.OUT)
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)

    def backward(self):
        GPIO.output(self.in1,GPIO.HIGH)
        GPIO.output(self.in2,GPIO.LOW) 

    def forward(self):
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.HIGH) 

    def stop(self):
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW) 

class moveMotor:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.rear_right = setMotor(25, 8, "rear_right")
        self.rear_right.setup()
        self.rear_left = setMotor(9, 11, "rear_left")
        self.rear_left.setup()
        self.front_left = setMotor(19, 26, "front_left")
        self.front_left.setup()
        self.front_right = setMotor(16, 20, "front_right")
        self.front_right.setup()
    
    def cleanup(self):
        GPIO.cleanup()
    
    def moveReset(self):
        self.rear_right.stop()
        self.rear_left.stop()
        self.front_left.stop()
        self.front_right.stop()
    
    def moveForward(self):
        self.rear_right.forward()
        self.rear_left.forward()
        self.front_left.forward()
        self.front_right.forward()
        self.state = 1
        rospy.sleep(0.25)
        self.moveStop()
    
    def moveBackward(self):
        self.rear_right.backward()
        self.rear_left.backward()
        self.front_left.backward()
        self.front_right.backward()
        self.state = 2
        rospy.sleep(0.25)
        self.moveStop()

    def moveStop(self):
        self.rear_right.stop()
        self.rear_left.stop()
        self.front_left.stop()
        self.front_right.stop()
        self.state = 0
    
    def turnLeft(self):
        self.rear_right.forward()
        self.rear_left.backward()
        self.front_left.backward()
        self.front_right.forward() 
        self.state = 3
        rospy.sleep(0.25)
        self.moveStop()
    
    def turnRight(self):
        self.rear_right.backward()
        self.rear_left.forward()
        self.front_left.forward()
        self.front_right.backward() 
        self.state = 4
        rospy.sleep(0.25)
        self.moveStop()
    
    def motorListener(self):
        rospy.init_node('mini_rover_motors')
        rospy.Subscriber('/rover_twist_fake', Twist, self.motorCallback)
        rospy.spin()   # see if I can change frequency to slower than inertial sense provision

    def testMotors(self):
        print("Testing motors...")

        print("Testing front left motor")
        print('FORWARD')
        self.front_left.forward()
        rospy.sleep(1)
        print('BACKWARD')
        self.front_left.backward()
        rospy.sleep(1)
        self.front_left.stop()

        print("Testing front right motor")
        print('FORWARD')
        self.front_right.forward()
        rospy.sleep(1)
        print('BACKWARD')
        self.front_right.backward()
        rospy.sleep(1)
        self.front_right.stop()

        print("Testing back left motor")
        print('FORWARD')
        self.rear_left.forward()
        rospy.sleep(1)
        print('BACKWARD')
        self.rear_left.backward()
        rospy.sleep(1)
        self.rear_left.stop()

        print("Testing back right motor")
        print('FORWARD')
        self.rear_right.forward()
        rospy.sleep(1)
        print('BACKWARD')
        self.rear_right.backward()
        rospy.sleep(1) 
        self.rear_right.stop() 

        self.cleanup()      


    def motorCallback(self, twist):
        # target angle must always be given relative to the 0 degrees for the robot 

        if twist.linear.y == 1 and twist.angular.z == 0:
            print("Move Forward")
            self.moveForward()
        if twist.linear.y == -1 and twist.angular.z == 0:
            print("Move Backward")
            self.moveBackward()
        if twist.angular.z == 1 and twist.linear.y == 0:
            print("Turn Right")
            self.turnRight()
        if twist.angular.z == -1 and twist.linear.y == 0:
            print("Turn Left")
            self.turnLeft()
        if twist.angular.z == 0 and twist.linear.y == 0:
            print("Stopping")
            self.moveStop()
        else:
            self.moveForward()
            
# set up for testing not as an encoder and motor node right now    
if __name__ == '__main__':
    rover_move = moveMotor()
    while not rospy.is_shutdown():
        rover_move.motorListener() 
    rover_move.cleanup() 