#ifndef UR_SIM_H
#define UR_SIM_H

#include <ros/ros.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>

#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include "ur_msgs/IOStates.h"

class UR_planning {

  moveit_msgs::PlanningScene p_scene;
  moveit_msgs::CollisionObject gripper;
  moveit_msgs::AttachedCollisionObject item, obs, item_visualization;
  moveit_msgs::CollisionObject collar1, collar2, collar3, collar4, box5;
  moveit::planning_interface::MoveGroupInterface group;
  moveit::planning_interface::MoveGroup::Plan my_plan;
  ros::NodeHandle n;
  ros::Publisher planning_scene_diff_publisher;
  ros::ServiceClient client;
  ros::Subscriber sub_wrench;
  ros::Subscriber sub_pose;
  ros::Subscriber sub_vacuum;
  bool force_mode;
  int force_mode_slow;

  public:
  int log_counter = 0;
  ur_msgs::IOStates io_states;

  UR_planning();
  int initPlanningScene();
  int attachItem();
  int item_vis(double x, double y,double z);
  int detachItem(bool remove = true);
  void set_item_level(double x, double y,double z, bool set);
  int moveTo(geometry_msgs::Pose pose);
  int moveTo_joints(std::vector<double> joints);
  std::vector<double> get_joints();
  int load_trajectory(std::string const &data_fname);
  int suction(bool on_off);
  int light(uint8_t intensity);
  void setForceMode(int fm);
  std::vector<bool> vacuum_status();

  private:
  bool planTrajectory(int numOfTries);
  void wrenchCallback(geometry_msgs::WrenchStamped msg);  
  void poseLog(sensor_msgs::JointState msg);
  void vacStat(ur_msgs::IOStates msg);
};

#endif // UR_SIM_H
