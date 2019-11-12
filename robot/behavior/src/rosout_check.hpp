#ifndef ROSOUT_CHECK_HPP
#define ROSOUT_CHECK_HPP

#include <iostream>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

#include <ignition/transport.hh>
#include <skoda_msgs/msgs/commandmsg.pb.h>
#include <skoda_msgs/msgs/errormsg.pb.h>
#include <skoda_msgs/msgs/statusmsg.pb.h>
using skoda_msgs::msgs::CommandMsg;
using skoda_msgs::msgs::ErrorMsg;
using skoda_msgs::msgs::InfoMsg;
using skoda_msgs::msgs::StatusMsg;

class RosoutCheck {
  ros::NodeHandle n;
  ros::Subscriber rosout_sub;

  ignition::transport::Node node;
  ignition::transport::Node::Publisher pub_error;
  ignition::transport::Node::Publisher pub_status;
  ignition::transport::Node::Publisher pub_info;

public:
  RosoutCheck();
  ~RosoutCheck();

  bool publish_err(skoda_msgs::msgs::ErrorMsg::Error_id id, std::string msg);
  bool publish_info(skoda_msgs::msgs::InfoMsg::Info_id id,
                    skoda_msgs::msgs::InfoMsg::Info_type type, std::string msg);

private:
  void rosoutCallback(rosgraph_msgs::Log msg);
};

#endif // !ROSOUT_CHECK_HPP
