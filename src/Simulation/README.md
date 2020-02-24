#Rover Gazebo Installation

##Rocker Bogie

#Installation
1. clone syrianspock into the workspace src folder `git clone https://github.com/SyrianSpock/rover.git`
2. remove the `rover` folder from the clone directory
3. Download `sudo apt-get install ros-melodic-hector-models`
4. Source the workspace and test by running `roslaunch rover_simulation simulation.launch`
5. Replace the `rover.urdf.xacro` file with the one here
