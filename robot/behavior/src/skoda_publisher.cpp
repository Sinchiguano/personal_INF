#include "skoda_publisher.hpp"




SkodaPublisher::SkodaPublisher() {

  std::string error_topic = "/errors";
  pub_error = node.Advertise<skoda_msgs::msgs::ErrorMsg>(error_topic);
  if (!pub_error) {
    std::cerr << "Error advertising topic [" << error_topic << "]" << std::endl;
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
}

SkodaPublisher::~SkodaPublisher() {}


bool SkodaPublisher::publish_err(skoda_msgs::msgs::ErrorMsg::Error_id id, std::string msg){
      skoda_msgs::msgs::ErrorMsg error_msg;
      error_msg.set_id(id);
      error_msg.set_msg(msg);
      if (!pub_error.Publish(error_msg)) {
        return false;
      }
      return true;
}

bool SkodaPublisher::publish_info(skoda_msgs::msgs::InfoMsg::Info_id id, 
    skoda_msgs::msgs::InfoMsg::Info_type type, std::string msg){

      skoda_msgs::msgs::InfoMsg info_msg;
      info_msg.set_id(id);
      info_msg.set_type(type);
      info_msg.set_msg(msg);
      if (!pub_info.Publish(info_msg)) {
        return false;
      }
      return true;
}