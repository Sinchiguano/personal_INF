#!/usr/bin/env bash

#source /home/binpicking/.bashrc
source /opt/ros/kinetic/setup.bash
source /home/binpicking/catkin_ws/devel/setup.bash
source /etc/profile.d/CameraSuiteEnv.sh

/opt/ros/kinetic/bin/roslaunch ur_sim.launch task:=grasping pn:=true
