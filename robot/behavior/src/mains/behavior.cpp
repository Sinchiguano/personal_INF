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
  
  ros::init(argc, argv, "behave");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~");

  ////////////////////////////////////////
  //please, either you wanna run the program in make data or graspping mode, execute 
  // the following command 
  //roslaunch perception_cvut bin_picking task:=grasping or make_data


  //Added by Cesar Sinchiguano from Ecuador je je je
  std::string task;
  if (ros::param::has("/grasping")) {
      ros::param::get("/grasping", task);
  }else{
    if (ros::param::has("/make_data")){
      ros::param::get("/make_data", task);
    }
  }
  ROS_INFO("Task: %s", task.c_str());
  /////////////////////////////////////////////////////

  //std::string task = "grasping";
  //std::string task = "make_data";



  bool pn = false;//Profinet is no longer used or supported
  nh.param<std::string>("task", task, "debug");
  nh.param<bool>("pn", pn, false);
  pn = false; 
  auto cmd = parseRobotCommand(task);// RobotCommand::grasping;//

  behave(cmd, pn);

  ros::Rate r(10);

  while(ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}

