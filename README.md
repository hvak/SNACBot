# SNACBot
SNACBot is a 5-axis robot arm that autonomously feeds people snacks. It uses computer vision via a gripper-mounted depth-sensing camera (Intel Realsense D435) to detect food and open mouths and calculate poses for each. More details can be found in [report.pdf](report.pdf).

![SNACBot](snacbot.PNG)

[SNABot Demo](https://www.youtube.com/watch?v=f-vsUyTJYIk&ab_channel=ThomasKrolikowski)

## Repo Structure
- `face_detection/` - ROS Service to extract open mouth pose
- `snacbot_common/` - Reusable code & moveit interfaces
- `snacbot_controller/` - Dynamixel servo controller configs, lauch files; MoveIt action server clients
- `snacbot_description/` - URDF and mesh data that describes the arm
- `snacbot_machine/` - contains main state machine script and main lauch files
- `snacbot_moveit/` - MoveIt configuration package
- `snacbot_snacbot_arm_ikfast_plugin/` - Inverse kinematics solution plugin for MoveIt (IKFast with TranslationDirection5D IK Type)
- `snacket/` - ROS service for detecting snacks and calculating grasp pose (utilizes YOLOv5)

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
$ cd ~/catkin_ws
$ catkin build
```

## Run
In separate terminals, run:
```
$ roslaunch snacbot_machine snacbot.launch
$ roslaunch snacbot_controller dynamixel_controller.launch
$ roslaunch snacbot_machine state_machine.launch
```

## Contributors
- [Hersh Vakharia](https://github.com/hvak)
- [Audrey Cooke](https://github.com/audeophilic)
- [Tom Krolikowski](https://github.com/tkroliko)