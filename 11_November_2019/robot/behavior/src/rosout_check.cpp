#include "rosout_check.hpp"

RosoutCheck::RosoutCheck() {
      //Ignition publishers
    std::string error_topic = "/errors";
    pub_error = node.Advertise<skoda_msgs::msgs::ErrorMsg>(error_topic);
    if (!pub_error) {
      std::cerr << "Error advertising topic [" << error_topic << "]" <<
    std::endl;
    }

    std::string status_topic = "/status";
    pub_status = node.Advertise<skoda_msgs::msgs::StatusMsg>(status_topic);
    if (!pub_status) {
      std::cerr << "Status advertising topic [" << status_topic << "]"
                << std::endl;
    }

    std::string info_topic = "/info";
    pub_info = node.Advertise<skoda_msgs::msgs::InfoMsg>(info_topic);
    if (!pub_info) {
      std::cerr << "Info topic [" << info_topic << "]" << std::endl;
    }

      //rosout subscriber
      rosout_sub = n.subscribe("/rosout",1, &RosoutCheck::rosoutCallback, this);
      
}

RosoutCheck::~RosoutCheck() {}

bool RosoutCheck::publish_err(skoda_msgs::msgs::ErrorMsg::Error_id id,
                              std::string msg) {
  skoda_msgs::msgs::ErrorMsg error_msg;
  error_msg.set_id(id);
  error_msg.set_msg(msg);
  if (!pub_error.Publish(error_msg)) {
    return false;
  }
  return true;
}

bool RosoutCheck::publish_info(skoda_msgs::msgs::InfoMsg::Info_id id,
                               skoda_msgs::msgs::InfoMsg::Info_type type,
                               std::string msg) {

  skoda_msgs::msgs::InfoMsg info_msg;
  info_msg.set_id(id);
  info_msg.set_type(type);
  info_msg.set_msg(msg);
  if (!pub_info.Publish(info_msg)) {
    return false;
  }
  return true;
}

void RosoutCheck::rosoutCallback(rosgraph_msgs::Log msg) {

  if (msg.level == 4 && std::string(msg.name) == "/move_group") {

    // Proste nejaka blba chyba, kdy nema info o stavu robota
    // if (std::string(msg.msg) == "Failed to receive current joint state") {
    //   ROS_INFO("Neco se pokazilo :-(");
    //   std::cout << msg.msg << std::endl;
    //   RosoutCheck::publish_err(ErrorMsg::FTRCJS, "Failed to receive current joint state");
    // }

    // Nejspise protective stopped
    if (std::string(msg.msg) == "Controller handle  reports status FAILED") {
      ROS_INFO("Neco se pokazilo :-(");
      std::cout << msg.msg << std::endl;
      RosoutCheck::publish_err(ErrorMsg::PROTECTIVE_STOP, "Selhal pohyb robota - robot je nejspise bezpecnostne zabrzden");
    }
  }

  if (std::string(msg.name) == "/ur_driver") { // msg.level == 8 &&

    // protective stopped
    if (std::string(msg.msg) ==
        "Cannot accept new trajectories: Robot is protective stopped") {
      ROS_INFO("Robot is protective stopped!");
      std::cout << msg.msg << std::endl;
      RosoutCheck::publish_err(ErrorMsg::PROTECTIVE_STOP, "Robot je bezpecnostne zabrzden");
    }
  }
}
