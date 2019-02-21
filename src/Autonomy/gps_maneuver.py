#!/usr/bin/env python
# ROS imports
import rospy
from inertial_sense.msg import GPS
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Twist
# Utility imports
import math

# Rover Class
class Rover:
    # ---> Forward
    
    def __init__(self, latErrorThreshold, lonErrorThreshold, target):
        # Destination reached if errors are smaller than following.
        self.latThreshold = latErrorThreshold
        self.lonThreshold = lonErrorThreshold
        # Initializes rover speed. Values are arbitrary now. To be adjusted by testing.
        self.linearVelocity = 0.3
        self.referenceAngularVelocity = 1    # Permitted angular velocity -> [-1,1]
        self.angularVelocity = 1
        # Target latlon.
        self.targetLatLon = target
        # Useful Flags
        self.destinationReached = False
        # Initialize rover head-latlon. Run fails otherwise.
        self.latitude = 0
        self.longitude = 0
        self.heading = 0

    # Pulls GPS and Magnetometer values from the INS.
    def listenerPublisher(self):         
        rospy.Subscriber('/gps', GPS, self.callbackGPS)
        rospy.Subscriber('/mag', MagneticField, self.callbackMAG)
        self.pub = rospy.Publisher('drive', Twist, queue_size=10)
        self.velocity = Twist()
        # Sets up the rover and publishes to drive.
        while not (self.destinationReached or rospy.is_shutdown()):
            self.setupRover()
            self.driveRover()
            rospy.sleep(1)  # Sleeps for 1 second.

    # Stores GPS data.
    def callbackGPS(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude

    # Calculates rover heading w.r.t. True North.
    def callbackMAG(self, data):
        xMag = data.magnetic_field.x
        yMag = data.magnetic_field.y
        # Calculates the magnetic compass heading.
        if(yMag > 0):
            self.heading = 90 - math.degrees(math.atan2(xMag, yMag))
        elif(yMag < 0):
            self.heading = 270 - math.degrees(math.atan2(xMag, yMag))
        elif(yMag == 0 and xMag < 0):
            self.heading = 180
        else:
            self.heading = 0
        # Correction for declination
        # Toronto: -10.44; Hanksville Utah: +10.5
        # Add for -ve; Subtract for +ve
        self.heading = self.heading + 10.44

    # Calculates heading and latlon differences of rover from target.  
    # Returns true if destination reached, else returns false.
    # Calls motorController to calculate rover velocitites.
    def setupRover(self):
        # Get latlon of target.
        targetLat = self.targetLatLon[0]
        targetLon = self.targetLatLon[1]

        # latlon difference.
        latDiff = targetLat - self.latitude
        lonDiff = targetLon - self.longitude

        # Check if destination reached.
        if(abs(latDiff) < self.latThreshold and abs(lonDiff) < self.lonThreshold):
            self.destinationReached = True
            print("Made it!")
            return True

        # Calculates the angle in degrees from positive x-axis(->) counter-clockwise.
        targetAngle = math.degrees(math.atan2(latDiff, lonDiff))
        if(targetAngle < 0):
            targetAngle = 360 + targetAngle # Fixes negative values.

        # Changes head values to unit circle degree measurement with postive x-axis reference. 
        if (self.heading >= 0):
            if(self.heading <= 90):
                refinedHead = 90 - self.heading
            else:
                refinedHead = 360 - (self.heading - 90)
        else:
            refinedHead = 90 + abs(self.heading)

        # Angle to turn to point toward target.
        angleDifference = refinedHead - targetAngle

        # For angles to destination higher than 180, turn the other way, it's faster.
        if (angleDifference >= 180):
            angleDifference = -1 * (360 - angleDifference)
        elif (angleDifference <= -180):
            angleDifference = (360 - abs(angleDifference))

        # Call to motor controller.
        self.motorController(angleDifference)

        return False

    # Calculates appropriate velocities for a given angle difference.
    def motorController(self, angleDifference):
        if angleDifference >= 0:
            # Turn CCW.
            if angleDifference < 85:
                self.angularVelocity = self.referenceAngularVelocity * abs(math.sin(math.radians(angleDifference)))
                self.linearVelocity = 0.3 # Do cos(angle) if speed scaling needed.
            # Stop and turn for large difference.
            else:
                self.angularVelocity = self.referenceAngularVelocity
                self.linearVelocity = 0
        else:
            # Turn CW.
            if abs(angleDifference) < 85:
                self.angularVelocity = - self.referenceAngularVelocity * abs(math.sin(math.radians(angleDifference)))
                self.linearVelocity = 0.3
            # Stop and turn for large difference.
            else:
                self.angularVelocity = - self.referenceAngularVelocity
                self.linearVelocity = 0

    # Publish velocities to drive topic.
    def driveRover(self):
        self.velocity.linear.x = self.linearVelocity
        self.velocity.angular.z = self.angularVelocity
        self.pub.publish(self.velocity)
        
if __name__ == '__main__':
    print("Autonomous Driving!")

    # Acceptable Errors.
    latErrorThreshold = 0.00002
    lonErrorThreshold = 0.00002
    
    # Destination Coordinates.
    destination = [1, 1]

    # Autonomous Driving.
    rover = Rover(latErrorThreshold, lonErrorThreshold, destination)
    rospy.init_node('rover_autonomy', anonymous=True)
    rover.listenerPublisher()