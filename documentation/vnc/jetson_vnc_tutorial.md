# How to connect to RSX Jetson with VNC

#### VNC allows you to connect to the Jetson board remotely with a graphical user interface (xfce). 

#### Yifan Lu, Kyrel Jerome

January 2020

## System Requirement
- Any device with VNC software installed (computers, smart phones, tablets, etc.). We recommend using RealVNC Viewer [(Download link)](https://www.realvnc.com/en/connect/download/viewer/)

## How to Connect
- Make sure you are connected to the `rsx_rover_router` network. 
- Log in to the board with `ssh rsx@192.168.1.50`. The IP address may have changed by the time you read this tutorial. You can use `ifconfig` and look under `eth0` to find the current IP address. 
- Start a VNC session with `vncserver -geometry 1920x1080 -depth 24`. Note that to have the best experience, you can set the `geometry` parameter to your own screen resolution. 
- You should now see a message telling you that the session is ready. See example below. In this case, the display number assigned to you is `:2`. Note this number, as you will need it to connect to the board and also to close your session later. 
```
Warning: rsx-jetson:1 is taken because of /tmp/.X11-unix/X1
Remove this file if there is no X server rsx-jetson:1

New 'rsx-jetson:2 (rsx)' desktop is rsx-jetson:2

Starting applications specified in /home/rsx/.vnc/xstartup
Log file is /home/rsx/.vnc/rsx-jetson:2.log
```
- Next you can open RealVNC Viewer on your computer, and create a new connection. In the `VNC Server` field, enter `192.168.1.50:590x`. In this case, the IP address of the Jetson board is `192.168.1.50`. You will need to replace `x` in the port number `590x` with your display number. For example, a display number of `:2` would mean that the port number is `5902`. 

## Closing a Connection
- After you are done with the current session, you need to close the connection so the resource can be freed. 
- To do so, run `vncserver -kill :2` on the Jetson board. In this case, the display number is 2, you will need to replace it with your own display number. 
