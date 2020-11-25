#!/bin/bash

# Edit default values here
# Note recommended, as if you git push then changes will be reflected across all users
ARDUINO_DEFAULT=0
CONTROLLER_DEFAULT=2

quiet=0

# Parse optional args
case "$1" in
        -h|--help)
                echo -e "launch_drive bash script V.1 (2020 Nov 25, Eddie Tian)\n"
                echo -e "Usage: launch_drive [arguments]\n"
                echo "Arguments"
                echo -e "\t-h|--help               Show this help screen"
                echo -e "\t-c|--change <a> <c>     Temporarily change port for arduino to \"a\""
                echo -e "\t-q|--quiet              Don't run rqt"
                echo -e "\t                        and port for controller to \"c\"\n\n"
                #echo -e "\t--help            Same as -h"
                #echo -e "\t--change <a> <c>  Same as -c"
                echo "Background jobs:"
                echo -e "\t<Ctrl>-Z \t\tPause and put foreground job in background"
                echo -e "\tbg %<N>  \t\tMove job <N> to background"
                echo -e "\tfg %<N>  \t\tMove job <N> to foreground"
                echo -e "\tkill %<N>\t\tStop job <N>"
                echo -e "\tjobs     \t\tView current jobs and their task numbers"
                exit  ;;

        -c|--change)
                ARDUINO_DEFAULT= $2
                CONTROLLER_DEFAULT= $3
                echo "Ports have changed for this call:"
                echo -e "Arduino port:   \t$2"
                echo -e "Controller port:\t$3"
                ;;
        -q|--quiet)
                quiet=1
                ;;
esac

echo "Starting roscore..."
roscore &

echo "Setting up ros parameters for drive control..."
rosparam set joy_node/dev "/dev/input/js$CONTROLLER_DEFAULT" 

echo "Starting rosserial_python..."
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM$ARDUINO_DEFAULT _baud:=57600 &

echo "Starting drive_sender..."
rosrun rover drive_sender &

echo "Starting joy_node..."
rosrun joy joy_node &

if [$quiet=0]; then
        echo "Initializing rqt plugin: gauges..."
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

        echo "Finished initializing all rqt plugins"
        echo "Starting rqt..."
        rqt &
fi

#echo "\n Showing jobs:"
#jobs
