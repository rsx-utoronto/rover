#!/usr/bin/python3
import smbus		#import SMBus module of I2C
from time import sleep, time  #import sleep
import math  
import rospy
import csv

class Magnetometer:
    def __init__(self):
        '''
            Find Heading by using HMC5883L interface with Raspberry Pi using Python
            http://www.electronicwings.com
        '''


        #some MPU6050 Registers and their Address
        self.Register_A     = 0x0B              #Address of Configuration register A
        self.Register_B     = 0x0A           #Address of configuration register B
        self.Register_mode  = 0x09           #Address of mode register

        self.scale = 0.0092
        self.X_axis_H    = 0x00             #Address of X-axis MSB data register
        self.Z_axis_H    = 0x02             #Address of Z-axis MSB data register
        self.Y_axis_H    = 0x04              #Address of Y-axis MSB data register
        self.declination = -0.00669          #define declination angle of location where measurement going to be done
        self.pi          = 3.14159265359     #define pi value`
        self.bus = smbus.SMBus(1)
        sleep(1)
        self.Device_Address = 0x0d


    def setup(self):
        #write to Configuration Register A
        self.bus.write_byte_data(self.Device_Address, self.Register_A, 0b01110000) # 0x70

        #Write to Configuration Register B for gain
        self.bus.write_byte_data(self.Device_Address, self.Register_B, 0b00100000) # 0x20, was a0

        #Write to mode Register for selecting mode
        self.bus.write_byte_data(self.Device_Address, self.Register_mode, 0b00011101) # 0x1D, was 0
    
    def readRaw(self, addr):
        #Read raw 16-bit value
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from module
        if(value > 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    def magnetometerPublisher(self):
        #Read Accelerometer raw value
        x = self.readRaw(self.X_axis_H)*self.scale
        y = self.readRaw(self.Z_axis_H)*self.scale
        z = self.readRaw(self.Y_axis_H)*self.scale

        heading = math.atan2(y, x) + self.declination

        print(math.atan2(y,x))
        
        #Due to declination check for >360 degree
        if(heading > 2*self.pi):
                heading = heading - 2*self.pi

        #check for sign
        if(heading < 0):
                heading = heading + 2*self.pi

        #convert into angle
        heading_angle = int(heading * 180/self.pi)

        print ("Heading Angle = %d°" %heading_angle)
        sleep(1)

compassValuesList = []
x_all = 0
y_all = 0

if __name__ == '__main__':
    rospy.init_node("mini_rover_magnetometer")
    rover_magnetometer = Magnetometer()
    rover_magnetometer.setup()
    print("Reading Heading Angle: ")
    # while not rospy.is_shutdown():
    #     rover_magnetometer.magnetometerPublisher() 

    for i in range(0,500):
        x = rover_magnetometer.readRaw(rover_magnetometer.X_axis_H)
        y = rover_magnetometer.readRaw(rover_magnetometer.Z_axis_H)
        z = rover_magnetometer.readRaw(rover_magnetometer.Y_axis_H)

        x_out_scaled = x * rover_magnetometer.scale
        y_out_scaled = y * rover_magnetometer.scale

        heading = math.atan2(y, x) + rover_magnetometer.declination
        
        #Due to declination check for >360 degree
        if(heading > 2*rover_magnetometer.pi):
                heading = heading - 2*rover_magnetometer.pi

        #check for sign
        if(heading < 0):
                heading = heading + 2*rover_magnetometer.pi

        #convert into angle
        heading_angle = int(heading * 180/rover_magnetometer.pi)
        
        print(heading_angle)
        
        x_all += x
        y_all += y
    
        compassValues = {}
        compassValues[ "X" ] = x
        compassValues[ "Y" ] = y
        compassValues[ "X_scaled" ] = x_out_scaled
        compassValues[ "Y_scaled" ] = y_out_scaled
        compassValues[ "Bearing" ] = heading
        compassValuesList.append( compassValues )

        sleep(0.1)
    
    print(x_all/500)
    print(y_all/500)

    outputFilename = "Compass_values_{0}.csv".format( int( time() ) )
    with open( outputFilename, "w" ) as csvFile:
        dictWriter = csv.DictWriter ( csvFile,[ "X", "Y", "X_scaled", "Y_scaled", "Bearing"] )
        dictWriter.writeheader()
        dictWriter.writerows( compassValuesList )