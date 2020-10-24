HOW TO RUN DRIVE
hi
Pull latest version of the drive board branch of rover
Open 4 terminal windows and run the following commands in each in the given order.

1. roscore
2. rosrun rosserial_python serial_node.py _port:=/dev/ttyACM(0 or 1) _baud:=57600

  //The port is usually ACM0 for the mega, sometimes ACM1.

3. rosrun rover drive_sender
4. rosrun joy joy_node

# Rover Software
ROS Package: Melodic

Package name: rover

To clone this package:

1) Create a folder (name doesn't matter)
2) Create a src folder inside
3) git clone this repository into this src folder
4) cd ../..
5) catkin_make

To run this package:

1) Compile the package by running this command in the project folder (the folder you created):
  catkin_make
2) Run each nodes by typing in this command:
  rosrun rover "name of the node"
  
---To SSH into the Nvidia Jetson---
1. Connect to the router wi-fi: (possibly rsx_rover_router, password: rsx123rsx123)
2. Use the following command:
   command: ssh rsx@192.168.1.50 (Jetson ip address)
   password: rsx123
---If you're not able to SSH into the Jetson (emergency procedure)----
1. Connect to rsx_rover_router.
2. Go to this link: http://whatsmyrouterip.com/
3. Type in wi-fi IP address into search bar.
4. Get Jetson's IP address from device list.
5. change ssh command to reflect the new IP address.
---
---IP Settings to look into if there is a comm issue---
Go into Bashrc: ~/.bashrc
scroll to the bottom and find two lines: ROS_MASTER_URI and ROS_IP
set ROS_MASTER_URI to the Jetson ipAddress
set ROS_IP to the current device's ipAddress
type: ifconfig to find the current ip address

type: source ~/.bashrc to set the changes into effect
Restart all the terminals if it still doesn't work

---
---To run Drive---
Shortcut commands (works on the rsx laptop and the jetson): 
nvidia (ssh into nvidia)
core (start roscore)
drive1 (run drive receiver node)
drive2 (run drive sender node)

1) Upload the Drive_receiver Arduino code on the Arduino
2) ssh into the nvidia jetson and run roscore
4) Run the receiver node in the jetson using this command (***using your port number): rosrun rosserial_python serial_node.py _port:=/dev/ttyACM# _baud:=57600 or drive1
5) On the separate command window, use this command: rosrun rover drive_sender or drive2
6) Control the rover using the arrow keys

---
---To run SLAM (Autonomy)---

1) roscore
2) on Jetson (Or any device with the stereo camera and RTABMAP setup): roslaunch rover traverse.launch
3) on the groundstation computer: rosrun rviz rviz to visualize the rover

---
---To run ARM---
1) Install openrave. Follow instructions from https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html
2) Go to arm control folder
3) Run script with ./filename.ext (If it's not an executable, ensure "#!/Use/bin/env python" is at the top of the file and type "chmod +x filename.ext" in the terminal)

Openrave is mainly used for visualization, whilst the main control is done in the script.
To make a visual model of the arm follow these steps:
4) Make an arm model in a urdf file format. Follow this tutorial to assist you: http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file
5) Convert a urdf file into a collada file. Follow this page for assistance with that: http://wiki.ros.org/collada_urdf
6) In the collada (*.dae) file, due to some bug, manually change the arm links parameters with the help of the file Arm_visualization_params_manual.odt
7) Make sure that the collada file name is the same as in environment.xml
8) Your visual model is set now, enjoy!

To run using ROS, steps (make sure you have the joystick connected beforehand):
1) Make sure that your computer is configured properly. The catkin package should be made. In  ~/.bashrc file make sure you export correct ROS_MASTER_URI and ROS_IP (Note: be careful when using ROS_IP together with ROS_HOSTNAME, as they are mostly mutually exclusive)
2) to run the Master, type "roscore" in the command line
3) to run the Arm Listener, which listenes for the angle values (usually run on the rover computer), type "rosrun rover arm_listener_ROS.py"
4) to run the control script (usually run on the ground station computer controlling the arm), in the root of the cloned from github "rover" folder type "rosrun rover arm_control_RSX_2018_ROS.py"

For using the arm camera, go through the camera manual. Its IP is 192.168.0.10 (make sure the network is 192.168.*.*)
---


