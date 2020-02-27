HOW TO RUN DRIVE

Open 4 terminal windows and run 1 in each in the given order.

1. roscore

2. rosrun rosserial_python serial_node.py _port:=/dev/ttyACM(0 or 1) _baud:=57600

//The port is usually ACM0 for the mega, sometimes ACM1.

3. rosrun rover drive_sender

4. rosrun joy joy_node
