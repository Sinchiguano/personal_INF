#ifndef GUI_SUB_HPP
#define GUI_SUB_HPP

#include <string>
#include <iostream>

//#include <skoda_msgs/msgs/commandmsg.pb.h>

class GUISubscriber {
public:
  GUISubscriber();
  GUISubscriber(GUISubscriber &&) = default;
  GUISubscriber(const GUISubscriber &) = default;
  GUISubscriber &operator=(GUISubscriber &&) = default;
  GUISubscriber &operator=(const GUISubscriber &) = default;
  ~GUISubscriber();

private:
};

// inline void CommandMsgCallback(const CommandMsg& msg){
//   std::cerr << msg.msg() << std::endl;

// }

#include "posePub.hpp"

GUISubscriber::GUISubscriber() {
  publish_some_stuff();
//   const std::string command_topic_ = "/command";
//   if (!node.Subscribe(command_topic_, CommandMsgCallback)) {
//     std::cerr << "Error subscribing to topic [" << command_topic_ << "]"
//               << std::endl;
//   }
//   std::cout << "Subscribed to topic [" << command_topic_ << "]" << std::endl;
}

GUISubscriber::~GUISubscriber() {}

#endif // GUI_SUB_HPP