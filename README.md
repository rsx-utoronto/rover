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
  
  
