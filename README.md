## Overview

ROS Publisher/Subscriber 

## Dependencies
```
1. ROS melodic
2. Catkin
3. Ubuntu 10.04 LTS
```
Install ROS melodic and setup catkin workspace by following this tutrial:
1. [Link to ROS tutorial!](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

## Standard install via command-line
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/kartikv97/beginner_tutorials.git
cd ~/catkin_ws
source ./devel/setup.bash
catkin_make
```
Open a new terminal and run the commands given below:

1. Open terminal 1 and run:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  roscore
```  
2. Open terminal 2 and run:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  rosrun beginner_tutorials talker
```
  
3. Open terminal 3 and run:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  rosrun beginner_tutorials listener
```
To stop the program press Ctrl + c in each terminal 


