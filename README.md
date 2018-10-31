# Rover Software
All the software for the rover. Started using ROS

Package name: rover_erc

Nodes:
Drive_server_node: drive_server
Drive_client_node: drive_client (Arduino code)
... (Add more nodes)


To clone this package:

1) Create a folder (name doesn't matter)
2) Create a src folder inside
3) git clone this repository into this src folder

To run this package:

1) Compile the package by running this command in the base folder (the folder you created):
  catkin_make
2) Run each nodes by typing in this command:
  rosrun rover_erc "name of the node"
  
  
---To run Drive---

1) roscore 
2) Upload the Drive_receiver Arduino code on the Arduidno
3) Run the receiver node using this command (***using your port number): rosrun rosserial_python serial_node.py _port:=/dev/ttyACM# _baud:=57600
4) On the separate command window, use this command: rosrun rover_erc drive_sender
5) Control the rover using the arrow keys

---
---To run SLAM---

1) roscore
2) on Jetson (Or any device with the stereo camera and RTABMAP setup): roslaunch rover_erc traverse.launch
3) on the groundstation computer: rosrun rviz rviz

---
