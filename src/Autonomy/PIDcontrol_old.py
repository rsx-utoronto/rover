#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from inertial_sense.msg import GPS
from sensor_msgs.msg import MagneticField
import threading
import math
import time

class AutonomousRover:
# '''
         North
#         |
#        ---
#         |
# ^
# |   : lat, y (+)
#
# - >   : long, x (+)
#
# '''

    def __init__(self, tick, errorx, errory, server):
        # Time at start.
        self.tick = tick
        # Time when operation is stopped.
        self.stopTime = 300
        # Permissible errors, destination reached if value smaller than this.
        self.xError = errorx
        self.yError = errory
        # Initial speed. Arbitrary value. Adjusted by testing.
        self.lin_vel = 0.3
        self.ref_ang_vel = 1
      
        self.timeOut = False
        self.arrived = False

        self.m = 0
        self.b = 0

        self.prevError = 0
        self.accError = 0

        # Thread keeps subscriber running while rest of the code executes
        # Thread runs for infinite time
        t = threading.Thread(target = listener, name = 'listener_thread')
        t.start()

    def callbackGPS(data):
        print data  #for testing
        # If print doesn't work use loginfo
        # rospy.loginfo(rospy.get_name()+ "Coordinates are x=%f y=%f z=%f", data.pose.pose.position.x,
        #                                                                  data.pose.pose.position.y,
        #                                                                  data.pose.pose.position.z)
        self.latitude = data.latitude
        self.longitude = data.longitude

    def callbackMAG(data):
        print data  #for testing
        self.xMag = data.magnetic_field.x
        self.yMag = data.magnetic_field.y
        self.head = math.atan2(self.xMag, self.yMag)

    def listener():
        rospy.init_node('gps_listener', anonymous=True)
        rospy.Subscriber('/gps', GPS, callbackGPS)
        rospy.Subscriber('/mag', MagneticField, callbackMAG)
        rospy.spin()   #see if I can change frequency to slower than inertial sense provision
        
    def initializeController(self, coordinate)

        # Get initial coordinates of rover
        roverX = self.latitude
        roverY = self.longitude
        # Get target gps coordinates
        targetX = coordinate[0]
        targetY = coordinate[1]

        # Calculate line from rover to target
        self.m = (targetY - roverY) / (targetX - roverX)
        self.b = roverY - m*roverX
    
    def move_towards_gps_location(self, coordinate):
    
        # Get target gps coordinates
        targetX = coordinate[0]
        targetY = coordinate[1]

        # Get rover coordinates
        roverX = self.latitude
        roverY = self.longitude

        # Calculate difference to target
        xDiff = targetX - roverX
        yDiff = targetY - roverY

        #for testing
        print ("xDiff X")
        print (self.xDiff)
        print ("yDiff Y")
        print (self.yDiff)

        # Checks to see if rover reached target
        if (abs(xDiff)) < self.xError and abs(yDiff) < self.yError):
            print ("Destination Reached!")
            self.arrived = True
            return True

       # Calculate error between rover position and target line
        m_90 = -(1/self.m)
        b_90 = roverY - m_90*roverX
        x_closest = (self.b - b_90) / (m_90 - self.m)
        y_closest = self.m*x_closest + self.b
        error = sqrt((x_closest - roverX)^2 + (y_closest - roverY)^2)

        # PID controller
        Kp = 2
        Kd = 2
        Ki = 2
        self.accError += error
        angle_correction = -Kp*error -Kd*(error-self.prevError) -Ki*(self.accError) #TODO: fix sign convention, tune Kp
        # Update previous error term
        self.prevError = error

        print("angle correction: " + str(angle_correction))

        self.motor_controller(angle_correction)
        return False

    # Motor Controller
    def motor_controller(self, angle_correction):
        angular_velocity = angle_correction
        self.lin_vel = 0.3
       
        #print("Left speed: " + str(left_speed) + " Right speed: " + str(right_speed))
