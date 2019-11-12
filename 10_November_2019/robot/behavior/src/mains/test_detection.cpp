#include <iostream>
//#include <ignition/msgs.hh>
//#include <ignition/transport.hh>
#include <ros/ros.h>

#include "robot_behavior.hpp"
#include "robot_interface.hpp"
#include <rosgraph_msgs/Log.h>

int do_ign_stuff();



int main(int argc, char** argv) {
  //do_ign_stuff();
  
  ros::init(argc, argv, "test_detection");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~");

  test_detection();

  ros::Rate r(10);

  while(ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}

