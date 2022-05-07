#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from inertial_sense.msg import GPS
from sensor_msgs.msg import MagneticField
import threading
import math
import time
import geodesy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class DeadReckonAutonomousRover:
# '''
#         North
#         |"'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''  "
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

        self.init_x = 0 
        self.init_y = 0
        self.init_utm = False 
        
        self.x = 0
        self.y = 0 
        self.zone_current = 'unknown'
        self.zone_prev = 'unknown'
        self.z = 0
        self.band = 'unknown'
        self.roll = 0
        self.pitch = 0
        self.yaw = 0 

        self.pub_target_angle = rospy.Publisher("target_angle", Float64 , queue_size=10)

    def callbackGPS(data):
        # If print doesn't work use loginfo
        # rospy.loginfo(rospy.get_name()+ "Coordinates are x=%f y=%f z=%f", data.pose.pose.position.x,
        #                                                                  data.pose.pose.position.y,
        #                                                                  data.pose.pose.position.z)
        self.latitude = data.latitude
        self.longitude = data.longitude
    
    def callbackUTM(data):
        self.easting = data.easting
        self.northing = data.northing
        self.altitude = data.altitude
        self.zone_current = data.zone
        self.band = data.band
        if self.init_utm == False:
            self.init_x = self.northing
            self.init_y = self.easting
            self.init_utm = True 
        # if self.zone_current != self.zone_prev:
            

    def callbackMAG(data):
        self.xMag = data.magnetic_field.x
        self.yMag = data.magnetic_field.y
        self.head = math.atan2(self.xMag, self.yMag)

    def callbackIMU(data):
        self.orientation = data.orientation
        self.angular_velocity = data.angular_velocity
        self.linear_acceleration = data.linear_acceleration

    def callbackOdom(data):
        self.pose = data.pose
        self.twist = data.twist 

        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def listener():
        rospy.init_node('gps_listener', anonymous=True)
        rospy.Subscriber('/imu', Imu, callbackIMU)
        rospy.Subscriber('/ins', Odometry, callbackOdom)
        rospy.Subscriber('/fix', NavSatFix, self.callbackGPS)
        rospy.Subscriber('/target_gps', NavSatFix, self.callbackTarget)
        rospy.Subscriber('mag', MagneticField, self.callbackMAG)
        rospy.spin() 
        
    def callbackTarget(self, coordinate):
        # Get target gps coordinates
        targetX = coordinate.easting
        targetY = coordinate.northing

        # Calculate difference to target
        xDiff = targetX - self.easting
        yDiff = targetY - self.northing

        #for testing
        print ("xDiff X")
        print (self.xDiff)
        print ("yDiff Y")
        print (self.yDiff)

        # Checks to see if rover reached target
        if (abs(xDiff) < self.xError and abs(yDiff) < self.yError):
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
        if (self.yaw >= 0):
            if(self.yaw <= 90):
                refinedHead = 90 - self.yaw
            else:
                refinedHead = 360 - (self.yaw - 90)
        else:
            refinedHead = 90 + abs(self.yaw)

        angle_from_rover = refinedHead - target_angle

        if (angle_from_rover >= 180):
            angle_from_rover = -1 * (360 - angle_from_rover)
        elif (angle_from_rover <= -180):
            angle_from_rover = (360 - abs(angle_from_rover))

        print("target angle: " + str(target_angle))
        print("refined head: " + str(refinedHead))
        print("angle from rover: " + str(angle_from_rover))

        self.pub_target_angle(angle_from_rover)

        return False

if __name__ == '__main__':
    rospy.init_node('dead_reckon_autonomy', anonymous=True)
    tick = rospy.Time.now()
    errorx = 0.01
    errory = 0.01 
    deadreckon_auto = DeadReckonAutonomousRover(tick, errorx, errory)

    while not rospy.is_shutdown():
        deadreckon_auto.listener()  