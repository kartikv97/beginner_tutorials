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
  roslaunch beginner_tutorials talkerListener.launch frequency:=7
```
  
3. Open terminal 3 and run (use default publish frequency):
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  roslaunch beginner_tutorials talkerListener.launch
```

4. Open terminal 4 and run (to update string):
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  // execute one of the statement from terminal 2 or 3 above before executing the statements in this terminal. 
  rosservice call /UpdateString "This is the updated string:"
```
To stop the program press Ctrl + c in each terminal 


