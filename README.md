<!-- You know what's cool? Working code. You know what's cooler? Properly document working code. 

Quick Markdown tips:

Headings are denoted by #. 

# This is a level 1 heading

## This is a level 2 heading

### This is a level 3 heading

And so on...


A single line break doesn't do anything in Markdown. You need two in a row to create an actual 
new paragraph. 


*This is underlined* 

**This is bolded**


1. Start a list like this
2. And continue 
3. As far as you
4. Want

* Bulleted lists start 
* with
* stars

To make a link that says "click me" which goes to "https://google.com"

[click me](https://google.com)


For inline code sections, surround the code with `back ticks` (the unshifted version of ~)

For code blocks, use this:

```
This is a line of code
Here's another one
```

It even supports syntax highlighting! Just add the language name after the first triple ticks

``` python

def printer():
	print ("RSX is the best")
	return True
```

And finally, a horizontal line can be added with any of the following:

---
***
___

Happy documenting! -->

# Rover Software

ROS Package: Melodic

Package name: rover

To clone this package:

1. Create a folder (name doesn't matter)

2. Create a src folder inside
3. git clone this repository into this src folder
4. cd ../..
5. catkin_make

To run this package:

1. Compile the package by running this command in the project folder (the folder you created):
   `catkin_make`

2. Run each nodes by typing in this command:
   `rosrun rover {name of the node}`
  
## To SSH into the Nvidia Jetson

1. Connect to the router wi-fi: (possibly rsx_rover_router, password: rsx123rsx123)

2. Use the following command:
   command: ssh rsx@192.168.1.50 (Jetson ip address)
   password: rsx123

### If you're not able to SSH into the Jetson (emergency procedure)

1. Connect to rsx_rover_router
2. Go to this link: http://whatsmyrouterip.com/
3. Type in wi-fi IP address into search bar.
4. Get Jetson's IP address from device list.
5. change ssh command to reflect the new IP address.

---

### IP Settings to look into if there is a comm issue
Go into Bashrc: `~/.bashrc`
scroll to the bottom and find two lines: `ROS_MASTER_URI` and `ROS_IP`
set `ROS_MASTER_URI` to the Jetson ipAddress
set `ROS_IP` to the current device's ipAddress
type `ifconfig` to find the current ip address

type: `source ~/.bashrc` to set the changes into effect
Restart all the terminals if it still doesn't work

## To Run Drive (New)

Pull latest version of the drive board branch of rover
Open 4 terminal windows and run the following commands in each in the given order.

1. `roscore`

2. `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600`
   If the above doesn't work, it's probably because the Arduino is on a different port.  
   Change ttyACM0 to ttyACM1 and try that. If that doesn't work, you can check which port the Arduino is attached to by looking in the bottom right corner of the Arduino IDE.
3. `rosrun rover drive_sender`
4. `rosrun joy joy_node`

### Troubleshooting
If running `rosrun rover drive_sender` says the rover package is not found, close all terminal windows and run:
`source ~/catkin_ws/devel/setup.bash`

It may be convenient to run this automatically every time a new terminal window is opened by adding it to .bashrc by typing: 
`echo "source ~/catkin_ws/devel/setup.bash">>~/.bashrc`

If all the programs run properly but nothing is showing on screen, it is possible the controller 
has not been configured. Close all terminal windows, and type:
`roscore`
Then in a new window, type:
`rosparam set joy_node/dev "/dev/input/jsX"`
Where X is a number. This instructions for finding this value can be found [here](https://github.com/rsx-utoronto/rover/wiki/Drive-Control#running-joystick-drive).

## To run Drive (old)

Shortcut commands (works on the rsx laptop and the jetson): 

`nvidia` (ssh into nvidia)
`core` (start roscore)
`drive1` (run drive receiver node)
`drive2` (run drive sender node)

1. Upload the Drive_receiver Arduino code on the Arduino
2. ssh into the nvidia jetson and run roscore
3. Run the receiver node in the jetson using this command (**using your port number**): 
   `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM# _baud:=57600`
   Alternatively, use 
   `drive1`

4. On the separate command window, use this command: 
   `rosrun rover drive_sender`
   or
   `drive2`

5. Control the rover using the arrow keys

## To run SLAM (Autonomy)

1. roscore
2. on Jetson (Or any device with the stereo camera and RTABMAP setup): 
   `roslaunch rover traverse.launch`

3. On the groundstation computer: 
   `rosrun rviz rviz to visualize the rover`

## To run ARM

1. Install openrave. Follow [these instructions](https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html).
2. Go to arm control folder
3. Run script with `./filename.ext` 

   (If it's not an executable, ensure `#!/Use/bin/env python` is at the top of filename.ext and type `chmod +x filename.ext` in the terminal)

   Openrave is mainly used for visualization, whilst the main control is done in the script.

To make a visual model of the arm follow these steps:

1. Make an arm model in a urdf file format. Follow [this tutorial](http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file) to assist you.
2. Convert a urdf file into a collada file. Follow [this page](http://wiki.ros.org/collada_urdf) for assistance with that.
3. In the collada (\*.dae) file, due to some bugs, manually change the arm links parameters with the help of the file Arm_visualization_params_manual.odt
4. Make sure that the collada file name is the same as in environment.xml
5. Your visual model is set now, enjoy!

To run using ROS, steps (make sure you have the joystick connected beforehand):

1. Make sure that your computer is configured properly. The catkin package should be made. In  ~/.bashrc file make sure you export correct `ROS_MASTER_URI` and `ROS_IP` (Note: be careful when using `ROS_IP` together with `ROS_HOSTNAME`, as they are mostly mutually exclusive)
2. to run the Master, type `roscore` in the command line
3. to run the Arm Listener, which listenes for the angle values (usually run on the rover computer), type `rosrun rover arm_listener_ROS.py`
4. to run the control script (usually run on the ground station computer controlling the arm), in the root of the cloned from github "rover" folder type `rosrun rover arm_control_RSX_2018_ROS.py`


**For using the arm camera, go through the camera manual. Its IP is 192.168.0.10 (make sure the network is 192.168..)**


