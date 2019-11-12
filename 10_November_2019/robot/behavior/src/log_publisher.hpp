#ifndef LOG_PUBLISHER
#define LOG_PUBLISHER

#include <ignition/transport.hh>
#include <skoda_msgs/msgs/statusmsg.pb.h>
using skoda_msgs::msgs::InfoMsg;


class LoggerPub {
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher pub_;

public:
  void publish_msg(std::string message, int type) {
      skoda_msgs::msgs::InfoMsg info_msg;
      info_msg.set_id(InfoMsg::DUMMY);
      if (type == 1){
        info_msg.set_type(InfoMsg::T_DUMMY);
      }
      else{
        info_msg.set_type(InfoMsg::T_SILENT);
      }
      info_msg.set_msg(message);
      pub_.Publish(info_msg);
  };

  LoggerPub(std::string message) {
    std::string log_topic = "/logs";
    pub_ = node_.Advertise<skoda_msgs::msgs::InfoMsg>(log_topic);
    if (!pub_) {
      std::cerr << "Status advertising topic [" << log_topic << "]"
                << std::endl;
    }

    publish_msg(message, 1);
  }

  LoggerPub(std::string message, int type) {
    std::string log_topic = "/logs";
    pub_ = node_.Advertise<skoda_msgs::msgs::InfoMsg>(log_topic);
    if (!pub_) {
      std::cerr << "Status advertising topic [" << log_topic << "]"
                << std::endl;
    }

    publish_msg(message, type);
  }
  ~LoggerPub() {
    
  }
};


#endif //! LOG_PUBLISHER