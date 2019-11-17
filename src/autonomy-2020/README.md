# RSX 2020 Autonomy code
## Required parts
 - An Nvidia Jetson TX2
 - An Intel Realsense D435i RGB-D camera
 - A rover

## Installation instructions from scratch

### Install NVIDIA Jetpack
- Install a fresh copy of [NVIDIA Jetpack 3.2.1](https://developer.nvidia.com/embedded/jetpack-archive) on the Jetson TX2 using a host machine.

- It is not nessecary to install the SDK on the host machine.

- A good micro-usb cable and an ethernet connection with a DHCP server (router) from the host to the Jetson are required.

- The IP address of the Jetson can be found using a ping scan. Example: 
```
nmap -sn 192.168.1.*
```

- Wait for the installation of the SDKs to fully complete before doing the next steps.

### SSH into the Jetson

- ssh into the Jetson's IP address. Example: `ssh nvidia@192.168.1.50`

- The default credentials for a fresh Jetson are 
   - username: `nvidia`
   - password: `nvidia`

### Install realsense drivers

sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev
wget https://github.com/Kitware/CMake/releases/download/v3.15.5/cmake-3.15.5-Linux-x86_64.sh
chmod +x cmake-3.15.5-Linux-x86_64.sh
sudo ./cmake-3.15.5-Linux-x86_64.sh


https://github.com/jetsonhacks/buildLibrealsense2TX

### Run some useful commands
```
sudo apt update
sudo apt upgrade
sudo apt install nano htop
```

- Set the hostname in `/etc/hosts` and `/etc/hostname` to `rsx-jetson`

- It's a good idea to reboot now

### Set the host machine to host an NTP server

This is required for cases where the Jetson will be behind an HTTP proxy (such as at the University of Toronto) or will have no internet conenction (such as in the middle of the desert). So pretty much all the time.

**These commands should be run on the host machine, not the Jetson.**

- Install packages and allow incomming ntp traffic on the host machine
```
sudo apt install ntp ntpdate
sudo ufw allow from any to any port 123 proto udp
```

- Add the following lines to `/etc/ntp.conf`
```
broadcast 192.168.1.255 minpoll 1
server 127.127.1.0
fudge 127.127.1.0 stratum 10
```

- Restart the NTP service to accept the changes
```
sudo service ntp restart
```

### Set up the Jetson to use the host machine as it's NTP server

- Install NTP on the Jetson
```
sudo apt install ntp
```

- Uncomment the following lines in `/etc/ntp.conf`
```
disable auth
broadcastclient
```

- Restart the NTP service to accept the changes
```
sudo service ntp restart
```

- Set the Jetson's timezone, replacing `America/Toronto` with the appropriate region

```
timedatectl list-timezones
sudo timedatectl set-timezone America/Toronto
```

### Install ROS and RealSense drivers on the Jetson

- Install `ros-kinetic-ros-desktop-full`  and set up the environment using the [instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu) on the ROS wiki.

- Install `ros-kinetic-ddynamic-reconfigure`
```
sudo apt install ros-kinetic-ddynamic-reconfigure
```

- Install the `realsense2_camera` package using the instructions [here](https://github.com/IntelRealSense/realsense-ros#installation-instructions). You've already completed Step 1 and 2.
