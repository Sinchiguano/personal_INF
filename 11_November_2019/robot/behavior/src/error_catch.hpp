#ifndef ERROR_CATCH
#define ERROR_CATCH

#include <thread>

#include <ignition/transport.hh>
#include <skoda_msgs/msgs/commandmsg.pb.h>
#include <skoda_msgs/msgs/errormsg.pb.h>
#include <skoda_msgs/msgs/statusmsg.pb.h>
using skoda_msgs::msgs::CommandMsg;
using skoda_msgs::msgs::ErrorMsg;
using skoda_msgs::msgs::InfoMsg;
using skoda_msgs::msgs::StatusMsg;

struct ErrorPublisher {
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher pub_;

  ignition::transport::Node::Publisher
  init_publisher(ignition::transport::Node &node) {
    std::string status_topic = "/errors";
    auto pub = node.Advertise<skoda_msgs::msgs::ErrorMsg>(status_topic);
    if (!pub) {
      std::cerr << "Status advertising topic [" << status_topic << "]"
                << std::endl;
    }
    return pub;
  }
  ErrorPublisher() : node_{}, pub_{init_publisher(node_)} {}
};

class TimeBomb {
  // int ms_;
  std::shared_ptr<bool> armed_;
  ErrorPublisher *pub_;
  std::thread bomb_thread_;

  std::thread init_thread(int ms, std::shared_ptr<bool> armed,
                          ErrorPublisher *pub, int type) {

    auto boom = [pub, type]() {
      skoda_msgs::msgs::ErrorMsg error_msg;
      error_msg.set_id(ErrorMsg::TIME_BOMB);
      if(type == 1){
        error_msg.set_msg("Selhani_planovace");
      }
      else if (type == 2){
        error_msg.set_msg("Selhani_kamery");
      }

      INFO("Movement timeout_error");
      pub->pub_.Publish(error_msg);
    };

    auto fn = [ms, armed, boom]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(ms));
      if (*armed) {

        boom();
      }
    };
    return std::thread(fn);
  }

public:
  TimeBomb(int ms, ErrorPublisher *pub, int type)
      : armed_{std::make_shared<bool>(true)},
        pub_(pub), bomb_thread_{init_thread(ms, armed_, pub_, type)} {
    bomb_thread_.detach();
  }

  ~TimeBomb() {
    *armed_ = false;
  }
};


class JointErr {
  // int ms_;
  ErrorPublisher *pub_;

public:
  void publish_err() {
      skoda_msgs::msgs::ErrorMsg error_msg;
      error_msg.set_id(ErrorMsg::JOINTS_OUT_OF_RANGE);
      error_msg.set_msg("Pozice kloubu robotu jsou mimo rozsah - srovnejte uhly robota do rozsahu -180 az 180 stupnu");
      INFO("Joints out of range - adjust joints in interval <-pi, pi>");
      pub_->pub_.Publish(error_msg);
  };

  JointErr(ErrorPublisher *pub)
      : pub_(pub){
    publish_err();
  }
  ~JointErr() {
  }
};


#endif //! ERROR_CATCH