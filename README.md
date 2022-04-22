# SNACBot
SNACBot is a 5-axis robot arm that autonomous feeds people food. 

## Build
ROS Version: Noetic  
Dependencies
- MoveIt (```sudo apt-get ros-noetic-moveit```)
- [realsense_ros](https://github.com/IntelRealSense/realsense-ros.git)
- [dynamixel_workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench.git) and [dynamixel_workbench_msgs](https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs)

Build SNACBot in your catkin workspace
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/hvak/SNACBot.git
$ cd ..
$ catkin build
```


## Run
In separate terminals, run:
```
$ roslaunch snacbot_machine snacbot.launch
$ roslaunch snacbot_controller dynamixel_controller.launch
$ roslaunch snacbot_machine state_machine.launch
```
