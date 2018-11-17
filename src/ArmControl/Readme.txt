1. Install Ubuntu 16.04
2. Install openrave. Follow instructions from https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html
3. install a Full-Desktop version of ROS Kinetic. Follow instructions from http://wiki.ros.org/kinetic/Installation/Ubuntu
4. Go to arm control folder
5. Run script with ./filename.ext (If it's not an executable, ensure "#!/Use/bin/env python" is at the top of the file and type "chmod +x filename.ext" in the terminal)

Openrave is mainly used for visualization, whilst the main control is done in the script.
To make a visual model of the arm follow these steps:
1. Make an arm model in a urdf file format. Follow this tutorial to assist you: http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file
2. Convert a urdf file into a collada file. Follow this page for assistance with that: http://wiki.ros.org/collada_urdf
3. In the collada (*.dae) file, due to some bug, manually change the arm links parameters with the help of the file Arm_visualization_params_manual.odt
4. Make sure that the collada file name is the same as in environment.xml
5. Your visual model is set now, enjoy!

To run using ROS, steps (make sure you have the joystick connected beforehand):
1. Make sure that your computer is configured properly. The catkin package should be made. In  ~/.bashrc file make sure you export correct ROS_MASTER_URI and ROS_IP (Note: be careful when using ROS_IP together with ROS_HOSTNAME, as they are mostly mutually exclusive)
2. to run the Master, type "roscore" in the command line
3. to run the Arm Listener, which listenes for the angle values (usually run on the rover computer), type "rosrun rover arm_listener_ROS.py"
4. to run the control script (usually run on the ground station computer controlling the arm), in the root of the cloned from github "rover" folder type "rosrun rover arm_control_RSX_2018_ROS.py"

For more info about how to communicate on network with ROS, google ROS Publishers, ROS Subscribers, ROS Topics, ROS Nodes


For using the arm camera, go through the camera manual. Its IP is 192.168.0.10 (make sure the network is 192.168.*.*)

