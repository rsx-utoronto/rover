#! /usr/bin/python

import rospy
from sensor_msgs.msg import NavSatFix
import csv

def callback(data):
    rospy.loginfo("Longitude: %f, Latitude %f, Altitude %f" % (data.longitude, data.latitude, data.altitude))

    f = open('gps.csv', 'a')

    # create the csv writer
    writer = csv.writer(f)

    # write a row to the csv file
    writer.writerow([data.longitude, data.latitude, data.altitude])
    f.close()
    
def start():

    rospy.init_node('gps_to_csv', anonymous=True)
    rospy.Subscriber("/fix", NavSatFix, callback)
    rospy.spin()

if __name__ == '__main__':
    start()





