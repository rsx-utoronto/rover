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
        self.pub = rospy.Publisher("encoder_state", Int32, queue_size=10)
        self.distance = 0
        self.radius = 0.0325 
    
    def setup(self):
        GPIO.setup(self.out, GPIO.IN)
    
    def readOut(self):
        print(GPIO.input(self.out))
    
    def count(self):
        self.cnt += 1
        if self.cnt % 20 == 0:
            self.rot += 1
    
    def encoderCallback(self):

        
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
                if stateCount == 20:
                    print("one rotation complete")
                    stateCount = 0
                    circum = 2*self.radius*PI
                    self.distance += circum  
                self.pub.publish(self.distance)
        except KeyboardInterrupt:
            GPIO.cleanup()
            exit()


 



    






        