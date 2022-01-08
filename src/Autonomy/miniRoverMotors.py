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
        self.state = 0
        self.pub_state = rospy.Publisher("motor_state", Int32, queue_size=10)
        self.pub_twist = rospy.Publisher("robot_twist", Twist, queue_size=10)

        # For larger rover implementation
        self.lin_vel = 0.3
        self.ref_ang_vel = 1

        self.arrived = False

        t = threading.Thread(target = motorListener, name = 'listener_thread')
        t.start()
    
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
        rospy.init_node('motor_heading_listener')
        rospy.Subscriber('/target_angle', , autoControlCallback)
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

    def manualControl(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            print("Key press?")
            press = 'q'
            press = input()
            if press == "w":
                self.moveForward()
                press == "s"
            elif press == "s":
                self.moveBackward()
                press == "s"
            elif press == "a":
                self.turnLeft()
                press == "s"
            elif press == "d":
                self.turnRight()
                press == "s"
            self.pub.publish(self.state)
            self.state = 0
            rate.sleep()
        self.cleanup()

    def autoControlCallback(self, angle_from_rover):
        # target angle must always be given relative to the 0 degrees for the robot 
        if angle_from_rover >= 0:
            #Turn right
            if angle_from_rover < 85:
                angular_velocity = self.ref_ang_vel * abs(math.cos(math.radians(abs(angle_from_rover))))
                self.lin_vel = 0.3
            #For large values just turn
            else:
                angular_velocity = self.reg_ang_vel
                self.lin_vel = 0 #stop and turn
        else:
            #Turn left
            if abs(angle_from_rover) < 85:
                self.lin_vel = 0.3
                angular_velocity = -self.ref_ang_vel * abs(math.cos(math.radians(abs(angle_from_rover))))
            #For large angles just turn
            else:
                angular_velocity = -self.reg_ang_vel
                self.lin_vel = 0 #stop and turn

        print("Left speed: " + str(left_speed) + " Right speed: " + str(right_speed))
            


# set up for testing not as an encoder and motor node right now    
if __name__ == '__main__':
    rospy.init_node('motor_state')
    rover_control = moveMotor()
    while not rospy.is_shutdown():
        
    rover_control.cleanup() 