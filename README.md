[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
### Turtlebot Navigation - A simulation of Turtlebot for obstacle avoidance

The turtlebot_nav package is a simple turtlebot navigator that detects obstacles and avoids them by rotating. 
The simulation is done on ROS Gazebo. 
![](images/turtlebot.PNG "Figure: Simulation on Gazebo")
## Dependencies
This project requires ROS Kinetic and catkin running on Ubuntu 16.04 LTS
To install ROS Kinetic:
http://wiki.ros.org/kinetic/Installation
To install catkin: 
It gets installed by default alongwith ROS
else:  http://wiki.ros.org/catkin
To install turtlebot packages
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
To install Gazebo
```
curl -sSL http://get.gazebosim.org | sh
gazebo
```
## Build Steps 
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make 
source devel/setup.bash
cd src/
git clone https://github.com/s-niket/turtlebot_nav
cd ..
catkin_make
source devel/setup.bash
```
## Running Demo
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch turtlebot_nav walker.launch
```
Press Ctrl+C to terminate

A .bag file will record all the published data. After running the Publisher/Subscriber node,we can view the 
full list of topics that are currently being published in the running system by running this in a new terminal:
```
rostopic list -v
```
The rosbag record with the option -a, indicating that all published topics should be accumulated in a bag file,
can be used to record all the published data. According to the launch file, the bag file will be created
in the /results directory under the name walkertopics.bag
```
rosbag record -a
```
To play back a .bag file, start roscore in one terminal and in other:
```
cd ~/catkin_ws/src/turtlebot_nav/results
rosbag play walkertopics.bag
```
To enable/disable recording, use the following commands:
```
roslaunch turtlebot_nav walker.launch rosbagEnable:=true
```
Note: During rosbag playback, Gazebo should'nt be running. 
