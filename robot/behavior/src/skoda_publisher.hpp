#ifndef SKODA_PUBLISHER_HPP
#define SKODA_PUBLISHER_HPP

#include <ignition/transport.hh>
#include <skoda_msgs/msgs/commandmsg.pb.h>
#include <skoda_msgs/msgs/errormsg.pb.h>
#include <skoda_msgs/msgs/statusmsg.pb.h>
using skoda_msgs::msgs::CommandMsg;
using skoda_msgs::msgs::ErrorMsg;
using skoda_msgs::msgs::InfoMsg;
using skoda_msgs::msgs::StatusMsg;

class SkodaPublisher {
  ignition::transport::Node node;
  ignition::transport::Node::Publisher pub_error;
  ignition::transport::Node::Publisher pub_status;
  ignition::transport::Node::Publisher pub_info;

public:
  SkodaPublisher();
  SkodaPublisher(SkodaPublisher &&) = default;
  SkodaPublisher(const SkodaPublisher &) = default;
  SkodaPublisher &operator=(SkodaPublisher &&) = default;
  SkodaPublisher &operator=(const SkodaPublisher &) = default;
  ~SkodaPublisher();

  bool publish_err(skoda_msgs::msgs::ErrorMsg::Error_id id, std::string msg);
  bool publish_info(skoda_msgs::msgs::InfoMsg::Info_id id, 
    skoda_msgs::msgs::InfoMsg::Info_type type, std::string msg);

private:
};



#endif // !SKODA_PUBLISHER_HPP
