#!/bin/bash

# Edit default values here
# Not recommended, as if you git push then changes will be reflected across all users
ARDUINO_DEFAULT=0
CONTROLLER_DEFAULT=2

# Make sure to update this list when new plugins are added
# See instructions at bottom
plugin_list=(gauges)


rqt=1
core=0

# Parse optional args
while [[ $# -gt 0 ]]
do
	key="$1"
	case $key in
		-h|--help)
			echo -e "launch_drive bash script V.1.1 (2021 Jan 16, Eddie Tian)\n"
			echo -e "Usage: launch_drive [arguments]\n"
			echo -e "Known Issue: Only way to stop nodes is with 'top' then 'kill {PID}'. A workaround is to close the terminal window, then run 'launch_drive' again."
			echo -e "Arguments"
			echo -e "\t-h|--help               Show this help screen"
			echo -e "\t-s|--set-port {a} {c}   Temporarily change port for arduino to \"a\""
			echo -e "\t                        Temporariily change port for controller to \"c\""
			echo -e "\t-n|--no-rqt             Don't run any rqt plugins. Default is run."
			echo -e "\t-c|--core               Run roscore (it's best to run roscore separately because you can't"
			echo -e "\t                        stop jobs started in scripts with \$jobs). Default is don't run."
			echo -e "\t-p|--plugin [plugins]   Start prompt to choose plugins to run. If not present, runs all plugins."
			echo -e "\t-l|--list-plugins       Show list of available plugins."

			# echo "Info on viewing background jobs:"
			# echo -e "\t<Ctrl>-Z \t\tPause and put foreground job in background"
			# echo -e "\tbg %<N>  \t\tMove job <N> to background"
			# echo -e "\tfg %<N>  \t\tMove job <N> to foreground"
			# echo -e "\tkill %<N>\t\tStop job <N>"
			# echo -e "\tjobs     \t\tView current jobs and their task numbers"
			exit 0
		;;
			-s|--set-port)
			ARDUINO_DEFAULT=$2
			CONTROLLER_DEFAULT=$3
			echo "Ports have changed for this call:"
			echo -e "Arduino port:   \t$2"
			echo -e "Controller port:\t$3"
			shift 2
			;;
		-n|--no-rqt)
			rqt=0
			;;
		-c|--core)
			core=1
			;;
		-l|--list-plugins)
			echo $plugin_list
			exit 0
			;;
		-p|--plugin)
			echo "Choose from this list of available plugins:"
			echo $plugin_list
			echo "Separate plugins with a space"
			read -a plugin
			plugin_list=${plugin}
			;;
		*)
			echo "Invalid argument. See launch_drive --help for help."
			exit 1
	esac
	shift
done


# Start roscore, if --core option set
if [ $core == 1 ]
then
	echo "Starting roscore..."
	roscore &
fi

# Start drive control
echo "Setting up ros parameters for drive control..."
rosparam set joy_node/dev "/dev/input/js$CONTROLLER_DEFAULT" 

# Start rosserial_python
echo "Starting rosserial_python..."
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM$ARDUINO_DEFAULT _baud:=57600 &

# Start drive_sender
echo "Starting drive_sender..."
rosrun rover drive_sender &

# Start joy_node
echo "Starting joy_node..."
rosrun joy joy_node &

# Don't run rqt if --no-rqt option set
if [ $rqt == 0 ]
then
	exit 0
fi


#Gauges plugin
if [[ " ${plugin_list[@]} " =~ "gauges" ]]; then
	echo "Initializing rqt plugin: gauges..."
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
	rosparam set /rqt_gauges/minimum2 -90
	rosparam set /rqt_gauges/maximum2 90
	
	rosrun rover gauge_receiver &
fi

# Format for adding new plugins:
# Copy and uncomment lines below, replacing YOUR_PLUGIN_NAME and YOUR_SETUP_CODE
# See gauges plugin for example
# Don't forget to add YOUR_PLUGIN_NAME to line 9

#if [[ " ${plugin_list[@]} " =~ "YOUR_PLUGIN_NAME" ]]; then
#	echo "Initializing rqt plugin: YOUR_PLUGIN_NAME"
#	YOUR_SETUP_CODE
#fi


echo "Finished initializing all rqt plugins"
echo "Starting rqt..."
rqt &
