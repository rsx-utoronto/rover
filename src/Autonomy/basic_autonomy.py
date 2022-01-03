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

        # Thread keeps subscriber running while rest of the code executes
        # Thread runs for infinite time
        t = threading.Thread(target = listener, name = 'listener_thread')
        t.start()

    def callbackGPS(data):
        print(data)  #for testing
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
        
    def move_towards_gps_location(self, coordinate):
    
        # Get target gps coordinates
        targetX = coordinate[0]
        targetY = coordinate[1]

        # Calculate difference to target
        xDiff = targetX - self.latitude
        yDiff = targetY - self.longitude

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

        # The Math.atan2() function returns the angle in the 
        # plane (in radians) between the positive x-axis and the ray from (0,0)
        # to the point (x,y), for Math.atan2(y,x). Converted to degrees
        target_angle = math.degrees(math.atan2(yDiff, xDiff))
        if (target_angle < 0):
            target_angle = 360 + target_angle

        # Sometimes the head values are messed up. (For old imu)
        #if(self.head > 180):
        #    self.head = -1 * (360 - self.head)
        #elif(self.head < -180):
        #    self.head = 360 + self.head

        # Heading starts from north. Clockwise is positive and CounterClockwise is negative.
        # Change it to unit circle degree measurement
        if (self.head >= 0):
            if(self.head <= 90):
                refinedHead = 90 - self.head
            else:
                refinedHead = 360 - (self.head - 90)
        else:
            refinedHead = 90 + abs(self.head)

        angle_from_rover = refinedHead - target_angle

        # For angles to destination higher than 180, turn the other way, because it's faster.
        if (angle_from_rover >= 180):
            angle_from_rover = -1 * (360 - angle_from_rover)
        elif (angle_from_rover <= -180):
            angle_from_rover = (360 - abs(angle_from_rover))

        print("target angle: " + str(target_angle))
        print("refined head: " + str(refinedHead))
        print("angle from rover: " + str(angle_from_rover))

        self.motor_controller(angle_from_rover)
        return False

    # Motor Controller
    def motor_controller(self, angle_from_rover):
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
