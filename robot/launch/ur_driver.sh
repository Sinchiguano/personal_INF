#!/usr/bin/env bash

source /opt/ros/kinetic/setup.bash
source /home/binpicking/catkin_ws/devel/setup.bash

#Skoda robot
#/opt/ros/kinetic/bin/roslaunch ur_modern_driver ur10_bringup_joint_limited.launch robot_ip:=192.168.0.10

#Lab robot
/opt/ros/kinetic/bin/roslaunch ur_modern_driver ur10_bringup_joint_limited.launch robot_ip:=192.168.1.201
