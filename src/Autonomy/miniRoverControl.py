#!/usr/bin/python3
import RPi.GPIO as GPIO 
from time import sleep
import rospy
from std_msgs.msg import Int32
import getch    

class Encoder:
    def __init__(self, vcc, gnd, out):
        self.vcc = vcc # 1
        self.gnd = gnd # 6
        self.out = out # 8
        self.cnt = 0
        self.rot = 0
    
    def setup(self):
        GPIO.setup(self.out, GPIO.IN)
    
    def readOut(self):
        print(GPIO.input(self.out))
    
    def count(self):
        self.cnt += 1
        if self.cnt % 20 == 0:
            self.rot += 1
    def test(self):
        stateLast = 0
        stateCount = 0
        stateCountTotal = 0
        try:
            while 1:
                stateCurrent = GPIO.input(self.out)
                if stateCurrent != stateLast:
                    print("current is not last")
                    print(stateCurrent)
                    stateLast = stateCurrent
                    stateCount += 1
                    stateCountTotal += 1
                print(stateCount)
        except KeyboardInterrupt:
            GPIO.cleanup()
            exit()

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
        self.rear_left = setMotor(9, 16, "rear_left")
        self.rear_left.setup()
        self.front_left = setMotor(19, 26, "front_left")
        self.front_left.setup()
        self.front_right = setMotor(16, 20, "front_right")
        self.front_right.setup()
        self.state = 0
        self.pub = rospy.Publisher("motor_state", Int32, queue_size=10)
    
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
    
    def manualControl(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print("Key press?")
            press = 'q'
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

    # def autoControl(self, target_angle):
    #     # target angle must always be given relative to the 0 degrees for the robot 

    #     if target_angle < 0:
            


    
if __name__ == '__main__':
    rospy.init_node('motor_state_publisher')
    rover_control = moveMotor()
    encoder = Encoder(0,0,14)
    encoder.setup()
    while not rospy.is_shutdown():
        rover_control.pub.publish(rover_control.state) 
        #encoder.test()
        rover_control.manualControl()
    rover_control.cleanup() 


 



    






        