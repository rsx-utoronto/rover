#!/bin/bash

# NOTE: This script must be launched on the jetson through ssh 

# Make sure to update this list when new plugins are added
# See instructions at bottom

# Setup environment 
cd ~/rover_ws
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.250:11311
export ROS_IP=192.168.0.69

# Do not start roscore unless specified
record=0
topic_list=()
CONTROLLER_DEFAULT=0
RVIZ_CONFIG="~/rover_ws/src/rover/scripts/launch/config/rviz"

# Parse optional args
while [[ $# -gt 0 ]]
do
	key="$1"
	case $key in
		-h|--help)
			echo -e "launch_drive bash script V.2.1 (2022 Aug 5, Catherine Glossop, based on work of Eddie Tian)\n"
			echo -e "Usage: launch_drive [arguments]\n"
			echo -e "Known Issue: Only way to stop nodes is with 'top' then 'kill {PID}'. A workaround is to close the terminal window, then run 'launch_drive' again."
			echo -e "Arguments"
			echo -e "\t-h|--help               Show this help screen"
			echo -e "\t-p|--port               Specify the port for the joystick of the form 'jsX' where X is specified here"
			echo -e "\t                        stop jobs started in scripts with \$jobs). Default is run."
			echo -e "\t-r|--record [topics]    Records a rosbag of the listed topics or all if a is given"
			echo -e "Each node is launched and a tmux panel is created for echoing topics and checking status of topics"
			exit 0
		;;
		-p|--port)
			CONTROLLER_DEFAULT=$2
			echo "Controller port has changed:"
			echo -e "Controller port:\t$2"
			shift 1
			;;
		-r|--record)
			echo "All topics recorded"
			echo "Seperate topics with a space"
			read -a record
			topic_list=${record}
			record=1
			;;
		*)
			echo "Invalid argument. See launch_drive --help for help."
			exit 1
	esac
	shift
done

# Start joy control
echo "Setting up ros parameters for drive control..."
rosparam set joy_node/dev "/dev/input/js$CONTROLLER_DEFAULT" 
echo "Starting joy node..."
rosrun joy joy_node &

# Start Visualization
echo "Starting up RVIZ..."
rosrun rviz rviz -d # config

if [ $record == 1 ]
then 
	echo "Recording rosbag..."
	echo "Warning! Rosbags can get very big. Please use sparingly."
	rosbag record -a
fi


tmux \
  new-session -s "drive" -n "control" \; \
  send-keys "rostopic echo drive" C-m \; \
  split-window -h -p 50 \; \
  send-keys "rostopic echo joy" C-m \; \
  new-window -n "motors" \; \
  send-keys "rostopic echo /front_left/status" C-m \; \
  split-window -v -p 33 \; \
  send-keys "rostopic echo /mid_left/status" C-m \; \
  split-window -v -p 33 \; \
  send-keys "rostopic echo /back_left/status" C-m \; \
  select-layout even-vertical \; \
  select-pane -t 0 \; \
  split-window -h\; \
  send-keys "rostopic echo /front_right/status" C-m \; \
  select-pane -t 2 \; \
  split-window -h \; \
  send-keys "rostopic echo /mid_right/status" C-m \; \
  select-pane -t 4 \; \
  split-window -h \; \
  send-keys "rostopic echo /back_right/status" C-m \; \
