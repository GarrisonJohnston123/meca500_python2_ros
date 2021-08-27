# Overview
This repository contains a ROS package for controlling a Mecademic meca500 r3 robot using ROS. The repo builds on the [ROS package](https://github.com/Mecademic/ROS) provided by the Mecademic company, but adds functionality beyond what is provided in their package. Additionally, this package is written in python 2.7 instead of 3.6 because ROS melodic [natively uses python 2.7](https://www.ros.org/reps/rep-0003.html#melodic-morenia-may-2018-may-2023) . I personally found it difficult to get their packages working well with ROS melodic. 

# Dependencies and Target Platforms
* Ubuntu 18.04
* Python 2.7 
* ROS Melodic 
* [meca500_python2_driver](https://github.com/GarrisonJohnston123/meca500_python2_driver) package

# Installation
## ROS Melodic
Follow the ROS melodic installation instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu)

## meca500_python2_driver
Follow the installation instructions [here](https://github.com/GarrisonJohnston123/meca500_python2_driver)

## meca500_python2_ros
In a command terminal run:
```
mkdir ~/meca_ws/src
cd ~/meca_ws/src
git clone https://github.com/GarrisonJohnston123/meca500_python2_ros.git
```
## Meca 500 r3 physical robot 
Follow the instructions given in section 4 of the [user manual](https://cdn.mecademic.com/uploads/docs/meca500-r3-user-manual-8-4.pdf)

# Getting Started
## Starting up the robot
Follow the startup proceedure given in section 6.1.1 of the [user manual](https://cdn.mecademic.com/uploads/docs/meca500-r3-user-manual-8-4.pdf)

## Starting the robot ROS driver
After the robot has been started up
```
cd ~/meca_ws
source devel/setup.bash
roslaunch mecademic_robot_node mecademic_robot_node.launch
```
## Running the example scripts
You can run an example script by running the following commands in a new command terminal
```
cd ~/meca_ws
source devel/setup.bash
rosrun mecademic_examples <example script name>.py
```
replace the `<example script name>` line with the name of an example script.

# Topics and Messages
The following table gives a list of the ROS topics used in this package. Note that the topic names have been changed from the original ros package to match the [ROS naming convention](http://wiki.ros.org/ROS/Patterns/Conventions#Naming_ROS_Resources) (see section 2.2) and for clarity. In general, topics ending in `_fb` are feedback from the robot (e.g., actual joint state) and topics ending in `_desired` are commands for the robot to execute. In general the `_fb` and `_desired` topics are the only ones needed.

| Topic Name | Message Type | Description |
|------------|--------------|-------------|
| mecademic_joint_desired | [sensor_msgs/JointState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html) | Desired joint state. For now, only the position is used. |
| mecademic_pose_desired | [geometry_msgs/Pose](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Pose.html) | Desired end-effector pose. |
| mecademic_joint_fb | [sensor_msgs/JointState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html) | The actual joint state. |
| mecademic_pose_fb| [geometry_msgs/Pose](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Pose.html) | The actual end-effector pose. |
| mecademic_gripper_desired | [std_msgs/Bool](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html) | Commands the robot to open or close the gripper. |
| mecademic_robot_command | [std_msgs/String](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) | TODO |
| mecademic_robot_reply | [std_msgs/String](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) | TODO |
| mecademic_robot_status | [std_msgs/UInt8MultiArray](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/UInt8MultiArray.html) | TODO |

# Example scripts
The following table gives a list of the example scripts provided by the `mecademic_examples` catkin package. See the Getting Started section for instructions on how to run the scripts.
| Example Name | Description |
|--------------|-------------|
|joystick_demo | A demo of using the robot with an xbox controller. You can use the direction pad to jog the end-effector in position while maintaining orientation. The A button sends the robot home. Face the end-effector for correct mapping between controller and robot. |