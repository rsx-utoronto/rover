HOW TO RUN DRIVE

Open 4 terminal windows and run 1 in each.

roscore

rosrun rosserial_python serial_node.py _port:=/dev/ttyACM(0 or 1) _baud:=57600

//The port is usually ACM0 for the mega, sometimes ACM1.

rosrun rover drive_sender

rosrun joy joy_node
