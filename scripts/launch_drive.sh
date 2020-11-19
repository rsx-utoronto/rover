#What it does
# starts drive control and any rqt modules associated with it
#Usage:
# $ launch_drive [controller number, usually 2] [arduino number, usually 0]
#!/bin/bash

# make sure you have roscore going!

#set the controller input number
rosparam set joy_node/dev "/dev/input/js$1" 

#start rosserial
# rosrun rosserial_python serial_node.py _port:=/dev/ttyACM$2 _baud:=57600 

#start other code
# rosrun rover drive_sender
# rosrun joy joy_node

#initialize rqt plugins

#gauges
# from wiki.ros.org/gauges:

#  "gauge_nameX" -- The name displayed below gauge X.
#  "minimumX" -- The minimum value shown on gauge X.
#  "maximumX" -- The maximum value shown on gauge X.
#  "danger_thresholdX" -- Show a color band, with red beginning at this value.
#  "topicX" -- The name of the topic that controls needle X.
#  "pixel_sizeX" -- The width and height of gaugeX.

rosparam set /rqt_gauges/topic1 /gauge_velocity_sender
rosparam set /rqt_gauges/gauge_name1 Velocity
rosparam set /rqt_gauges/minimum1 -100
rosparam set /rqt_gauges/maximum1 100

rosparam set /rqt_gauges/topic2 /gauge_angle_sender
rosparam set /rqt_gauges/gauge_name2 Angle
rosparam set /rqt_gauges/minimum2 -1
rosparam set /rqt_gauges/maximum2 1



# start rqt
# rqt
