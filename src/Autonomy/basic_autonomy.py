#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from inertial_sense_ros.msg import GPS
from sensor_msgs.msg import MagneticField
from geodesy.utm import UTMPoint, fromLatLong
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float64
import threading
import math
import time

### This is the only script that can be used with the mini rovers 

class BasicAutonomousRover:
# '''
#         North
#         |
#        ---
#         |
# ^
# |   : lat, y (+)
#
# - >   : long, x (+)
#
# '''
    def __init__(self, tick, errorx, errory):
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
        self.init_heading = 0
      
        self.timeOut = False
        self.arrived = False

        self.latitude = 0
        self.longitude = 0 
        self.altitude = 0
        self.head = 0

        self.targetX = self.latitude
        self.targetY = self.longitude
        self.utm = UTMPoint()

        self.pub_target_angle = rospy.Publisher("/target_angle", Float64 , queue_size=10)

    def callbackGPS(self, data):
        # If print doesn't work use loginfo
        # rospy.loginfo(rospy.get_name()+ "Coordinates are x=%f y=%f z=%f", data.pose.pose.position.x,
        #                                                                  data.pose.pose.position.y,
        #                                                                 data.pose.pose.position.z)
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.altitude = data.altitude

        self.utm = fromLatLong(self.latitude, self.longitude, self.altitude)

        # apply convert2UTM() if wanted 

    def callbackMAG(self, data):
        self.xMag = data.magnetic_field.x
        self.yMag = data.magnetic_field.y
        self.head = math.atan2(self.xMag, self.yMag)


    def listener(self):

        rospy.Subscriber('/fix', NavSatFix, self.callbackGPS)
        rospy.Subscriber('/target_gps', NavSatFix, self.callbackTarget)
        rospy.Subscriber('mag', MagneticField, self.callbackMAG)
        rospy.spin()   #see if I can change frequency to slower than inertial sense provision
        
    def callbackTarget(self, data): # gets actual heading between target and current GPS location
    
        # Get target gps coordinates
        self.targetX = data.latitude
        self.targetY = data.longitude
        self.targetZ = data.altitude

        # Calculate difference to target
        self.xDiff = self.targetX - self.latitude
        self.yDiff = self.targetY - self.longitude

        #for testing
        # print ("xDiff X")
        # print (self.xDiff)
        # print ("yDiff Y")
        # print (self.yDiff)

        # Checks to see if rover reached target
        if abs(self.xDiff) < self.xError and abs(self.yDiff) < self.yError:
            print ("Destination Reached!")
            self.arrived = True
            return True

        # The Math.atan2() function returns the angle in the 
        # plane (in radians) between the positive x-axis and the ray from (0,0)
        # to the point (x,y), for Math.atan2(y,x). Converted to degrees
        target_angle = math.degrees(math.atan2(self.yDiff, self.xDiff))
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
        time.sleep(1)
        print("-------------- CONTROL INFO ----------------")
        print("target angle: " + str(target_angle))
        print("refined head: " + str(refinedHead))
        print("angle from rover: " + str(angle_from_rover))
        print("--------------------------------------------")

        self.pub_target_angle.publish(angle_from_rover)

        return False

if __name__ == '__main__':
    rospy.init_node('basic_autonomy', anonymous=True)
    tick = rospy.Time.now()
    errorx = 0.00001
    errory = 0.00001 
    basic_auto = BasicAutonomousRover(tick, errorx, errory)
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        basic_auto.listener() 
        r.sleep()


