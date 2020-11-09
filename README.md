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

## Run launch file
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

## Verify TF frames 

Open a new terminal and run the commands given below:

1. Open terminal 1 and run: (talker node publishes tf frame /talk wrt. the parent /world)
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  rosrun rqt_tf_tree rqt_tf_tree
```  
2. Open terminal 2 and run: (The following command is used to echo the values.)
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  rosrun tf tf_echo /world /talk
```
3. Open terminal 3 and run: (view_frames creates a diagram of the frames being broadcast by tf over ROS. The generated pdf file is viewed using the evince command.)
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  rosrun tf view_frames
  evince frames.pdf
```
## Run Ros Unit Tests
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
  rostest beginner_tutorials test.launch
```
## Record bag file
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
  roslaunch beginner_tutorials talkerListener.launch startRosBagRec:=true
```
## Play bag file
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
  rosrun beginner_tutorials listener
```
3. Open terminal 3 and run:
```
  cd ~/catkin_ws/src/beginner_tutorials/results
  source ./devel/setup.bash
  rosbag play beginner_tutorials.bag
```
  