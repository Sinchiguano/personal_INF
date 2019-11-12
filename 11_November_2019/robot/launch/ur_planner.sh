#!/usr/bin/env bash
#bash -c "source /opt/ros/kinetic/bin/roslaunch ur_modern_driver ur10_bringup_joint_limited.launch robot_ip:=192.168.0.3"

source /opt/ros/kinetic/setup.bash
source /home/binpicking/catkin_ws/devel/setup.bash

/opt/ros/kinetic/bin/roslaunch real_ur10_moveit_planning_execution.launch
