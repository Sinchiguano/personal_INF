#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include <boost/sml.hpp>
#include <boost/variant.hpp>
#include <boost/variant/apply_visitor.hpp>

#include "camera.h"
#include "gige_camera.h"
#include "make_cam_pose_data.hpp"
#include "random_failure.hpp"
#include "real_robot.hpp"
#include "robot_client.h"
#include "robot_interface.hpp"
#include "skoda_publisher.hpp"

#include <chrono>
#include <thread>

#include "rosout_check.hpp"

#include "skoda_publisher.hpp"
#include <ignition/transport.hh>
#include <skoda_msgs/msgs/commandmsg.pb.h>
#include <skoda_msgs/msgs/errormsg.pb.h>
#include <skoda_msgs/msgs/statusmsg.pb.h>

using skoda_msgs::msgs::CommandMsg;
using skoda_msgs::msgs::ErrorMsg;
using skoda_msgs::msgs::InfoMsg;
using skoda_msgs::msgs::StatusMsg;

namespace sml = boost::sml;

int g_command = 0;
bool g_init = false;

struct robot_logger {
  template <class SM, class TEvent> void log_process_event(const TEvent &) {
    printf("[%s][process_event] %s\n", sml::aux::get_type_name<SM>(),
           sml::aux::get_type_name<TEvent>());
  }

  template <class SM, class TGuard, class TEvent>
  void log_guard(const TGuard &, const TEvent &, bool result) {
    printf("[%s][guard] %s %s %s\n", sml::aux::get_type_name<SM>(),
           sml::aux::get_type_name<TGuard>(), sml::aux::get_type_name<TEvent>(),
           (result ? "[OK]" : "[Reject]"));
  }

  template <class SM, class TAction, class TEvent>
  void log_action(const TAction &, const TEvent &) {
    printf("[%s][action] %s %s\n", sml::aux::get_type_name<SM>(),
           sml::aux::get_type_name<TAction>(),
           sml::aux::get_type_name<TEvent>());
  }

  template <class SM, class TSrcState, class TDstState>
  void log_state_change(const TSrcState &src, const TDstState &dst) {
    printf("[%s][transition] %s -> %s\n", sml::aux::get_type_name<SM>(),
           src.c_str(), dst.c_str());
  }
};

// events
struct e_init_home {};
struct e_init_manual {
  RobotCommand cmd;
};
struct e_init_localize {};
struct e_init_cycle {};
struct e_approach_fail {
  std::vector<GeometricTransformation> highest_objects;
};
struct e_start_approaching {
  std::vector<GeometricTransformation> highest_objects;
};
struct e_start_grasping {
  GeometricTransformation tfm;
  std::vector<GeometricTransformation> highest_objects;
};
struct e_start_putting {
  std::vector<GeometricTransformation> highest_objects;
};
struct e_grasp_fail {
  GeometricTransformation tfm;
  std::vector<GeometricTransformation> highest_objects;
};
struct other_event {};
struct e_normal {};

using AllEventsVariant =
    boost::variant<e_init_home, e_init_manual, e_init_localize, e_init_cycle,
                   e_normal, e_approach_fail, e_start_approaching,
                   e_start_grasping, e_start_putting, e_grasp_fail,
                   other_event>;

struct ControllerInterface {
  virtual void emit(AllEventsVariant event) = 0;
  virtual std::shared_ptr<RealRobot> get_robot_interface() = 0;
  virtual std::shared_ptr<SkodaPublisher> get_skoda_publisher() = 0;
};

struct CAdapter {
  ControllerInterface *c;
};

// guards
// no guards defined

// REACT TO COMMANDS
///////////////////////////////////
bool soft_stop() {
  if (g_command == 3) { // SOFT_STOP
    g_command = 0;
    return true;
  } else {
    // g_command = 0;
    return false;
  }
}

bool start_from_home() {
  if(g_init) {
    g_init = false;
    return true;
  }
  if (g_command == 2) { // SOFT_START
    g_command = 0;
    return true;
  } else
    return false;
}
///////////////////////////////////

// actions
struct a_go_home {
  void operator()(CAdapter &ca) {
    ca.c->get_skoda_publisher()->publish_info(
        InfoMsg::HOME, InfoMsg::T_OPERATION, std::string("Pocatecni pozice"));

    std::cout << "go home in action" << std::endl;
    ca.c->get_robot_interface()->a_go_home();

    ca.c->get_skoda_publisher()->publish_info(
        InfoMsg::ROBOT_OK, InfoMsg::T_ROBOT_STATUS, std::string("OK"));

    while (true) {
      //if i wanna start from home, cesar sinchiguano 
      //if (start_from_home())
      //cesar sinchiguano 
      if (true) 
      {
        ca.c->get_skoda_publisher()->publish_info(
            InfoMsg::FULL_BOX, InfoMsg::T_BOX_STATUS,
            std::string("Pocatecni stav"));
        ca.c->get_skoda_publisher()->publish_info(
            InfoMsg::LEAVING_HOME, InfoMsg::T_INFO,
            std::string("Opoustim pocatecni pozici"));
        ca.c->emit(e_init_cycle{});
      }

        ca.c->get_skoda_publisher()->publish_info(
            InfoMsg::WAITING_AT_HOME, InfoMsg::T_INFO,
            std::string("Pocatecni stav - cekam"));
      soft_stop(); // clear soft stop command
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
};
struct a_do_manual {
  void operator()(CAdapter &ca, auto e) {
    ca.c->get_skoda_publisher()->publish_info(InfoMsg::DUMMY, InfoMsg::T_SILENT,
                                              std::string("MANUAL"));

    std::cout << "START APPROACHING" << std::endl;
    ca.c->get_robot_interface()->a_do_manual(e.cmd);
  }
};
struct a_init_localize {
  void operator()(CAdapter &ca) {
    ca.c->get_skoda_publisher()->publish_info(InfoMsg::DUMMY, InfoMsg::T_SILENT,
                                              std::string("INIT SAFE MOVE"));
    ca.c->get_robot_interface()->init_safe_move();

    //ca.c->get_skoda_publisher()->publish_info(InfoMsg::DUMMY, InfoMsg::T_SILENT,
    //                                          std::string("LOCALIZE BOX"));
    //std::cout << "Localize box" << std::endl;
    //ca.c->get_robot_interface()->localize_box();
  }
};
struct a_making_initial_imgs {
  void operator()(CAdapter &ca) {

    //check if holding an item
    bool suction = ca.c->get_robot_interface()->init_check_suction();
    suction =false;


    if (soft_stop()) {
      ca.c->get_skoda_publisher()->publish_info(
          InfoMsg::SOFT_STOP, InfoMsg::T_INFO,
          std::string("Probiha zastaveni robota"));
      ca.c->emit(e_init_home{});
    }
    else if(suction){
      std::vector<GeometricTransformation> highest_objects;
      ca.c->emit(e_start_putting{highest_objects});
    } else {
      ca.c->get_skoda_publisher()->publish_info(
        InfoMsg::MAKE_INIT, InfoMsg::T_OPERATION,
        std::string("Prohledavani bedny"));

      // Take initial images and create a list of suitable objects
      std::vector<GeometricTransformation> highest_objects =
        ca.c->get_robot_interface()->a_making_initial_imgs();


      if (highest_objects.size() > 0) {
        // TODO: GET item level at the bottom
        double low_item_level = -0.29;
        if (highest_objects.at(0)(2, 3) < low_item_level) {
          // || (highest_objects.at(0)(2,3) < low_item_level + 0.2  &&
          // highest_objects.size() < 2)
          ca.c->get_skoda_publisher()->publish_info(
              InfoMsg::BOX_LOW, InfoMsg::T_BOX_STATUS,
              std::string("Dochazi soucastky"));
        } else {
          ca.c->get_skoda_publisher()->publish_info(
              InfoMsg::FULL_BOX, InfoMsg::T_BOX_STATUS,
              std::string("Dostatek soucastek"));
        }

        ca.c->emit(e_start_approaching{highest_objects});
      } else {
        ca.c->get_skoda_publisher()->publish_info(
            InfoMsg::EMPTY_BOX, InfoMsg::T_BOX_STATUS,
            std::string("Nebyly detekovany zadne soucastky"));
        std::cout << "No Items detected:" << std::endl
                  << "  - either the box is empty or there is a problem with "
                     "the camera"
                  << std::endl;
        ca.c->emit(e_init_home{});
      }
    }
  }
};
struct a_start_approaching {
  void operator()(CAdapter &ca, auto e) {
    ca.c->get_skoda_publisher()->publish_info(
        InfoMsg::MAKE_INIT, InfoMsg::T_OPERATION,
        std::string("Lokalizace soucastky"));

    GeometricTransformation tfm;

    std::string item_info = "Lokalizovane soucastky (" +
                            std::to_string(e.highest_objects.size()) + ")";
    ca.c->get_skoda_publisher()->publish_info(InfoMsg::DUMMY,
                                              InfoMsg::T_SILENT, item_info);

    // Use and then remove the highest object from the list
    bool success = ca.c->get_robot_interface()->a_start_approaching(
        e.highest_objects, tfm);
    e.highest_objects.erase(e.highest_objects.begin());

    if (soft_stop()) {
      ca.c->get_skoda_publisher()->publish_info(
          InfoMsg::SOFT_STOP, InfoMsg::T_INFO,
          std::string("Probiha zastaveni robota"));
      ca.c->emit(e_init_home{});
    } else {
      success ? ca.c->emit(e_start_grasping{tfm, e.highest_objects})
              : ca.c->emit(e_approach_fail{e.highest_objects});
    }
  }
};
struct a_recover_approaching {
  void operator()(CAdapter &ca, auto e) {

    if (soft_stop()) {
      ca.c->get_skoda_publisher()->publish_info(
          InfoMsg::SOFT_STOP, InfoMsg::T_INFO,
          std::string("Probiha zastaveni robota"));
      ca.c->emit(e_init_home{});
    } else {
      ca.c->get_skoda_publisher()->publish_info(
          InfoMsg::DUMMY, InfoMsg::T_SILENT,
          std::string("Approach_failed"));

      if (e.highest_objects.size() > 0) {
        ca.c->emit(e_start_approaching{e.highest_objects});
      } else {
        ca.c->emit(e_init_cycle{});
      }
    }
  }
};
struct a_recover_grasping {
  void operator()(CAdapter &ca, auto e) {
    if (soft_stop()) {
      ca.c->get_skoda_publisher()->publish_info(
          InfoMsg::SOFT_STOP, InfoMsg::T_INFO,
          std::string("Probiha zastaveni robota"));
      ca.c->emit(e_init_home{});
    } else {
      ca.c->get_skoda_publisher()->publish_info(
          InfoMsg::DUMMY, InfoMsg::T_SILENT,
          std::string("Grasping_failed"));
      ca.c->emit(e_approach_fail{e.highest_objects});
    }
  }
};
struct a_start_grasping {
  void operator()(CAdapter &ca, auto e) {
    ca.c->get_skoda_publisher()->publish_info(
        InfoMsg::GRASPING, InfoMsg::T_OPERATION,
        std::string("Uchopeni soucastky"));

    bool success = ca.c->get_robot_interface()->a_start_grasping(e.tfm);
    success ? ca.c->emit(e_start_putting{e.highest_objects})
            : ca.c->emit(e_grasp_fail{e.tfm, e.highest_objects});
  }
};
struct a_start_putting {
  void operator()(CAdapter &ca, auto e) {
    ca.c->get_skoda_publisher()->publish_info(
        InfoMsg::PUTTING, InfoMsg::T_OPERATION,
        std::string("Zakladani soucastky"));
    int success = ca.c->get_robot_interface()->a_start_putting();

    if (success == 1) {
      ca.c->get_skoda_publisher()->publish_info(
          InfoMsg::ONE_MORE_ITEM, InfoMsg::T_SILENT,
          std::string("Soucastka vlozena do zakladace"));
    }
    else if(success == 2){
      ca.c->get_skoda_publisher()->publish_info(
          InfoMsg::FEEDER_FULL, InfoMsg::T_SILENT,
          std::string("Sucastka volezena do zakladace - zakladac byl plny"));
    }
    else if (success == 3){
      ca.c->get_skoda_publisher()->publish_err(
          ErrorMsg::DROPPED,
          std::string("Zakladac je plny"));
                ca.c->get_skoda_publisher()->publish_info(
      InfoMsg::SOFT_STOP, InfoMsg::T_INFO,
          std::string("Probiha zastaveni robota"));
      ca.c->emit(e_init_home{});
    }
    else {
      ca.c->get_skoda_publisher()->publish_err(
          ErrorMsg::DROPPED_ITEM,
          std::string("Robot upustil soucastku pri zakladani"));
    }

    if (soft_stop()) {
      ca.c->get_skoda_publisher()->publish_info(
          InfoMsg::SOFT_STOP, InfoMsg::T_INFO,
          std::string("Probiha zastaveni robota"));
      ca.c->emit(e_init_home{});
    } else {
      if (e.highest_objects.size() > 0) {
        ca.c->emit(e_start_approaching{e.highest_objects});
      } else {
        ca.c->emit(e_init_cycle{});
      }
    }
  }
};

// event -> guard -> transition -> action
struct robot_world {
  auto operator()() const noexcept {
    using namespace sml;
    return make_transition_table(
        // clang-format off
            "home"_s <= *"init"_s + event<e_init_home> / a_go_home{},
            "manual"_s <= "init"_s + event<e_init_manual> / a_do_manual{},
            "localize_box"_s <= "init"_s + event<e_init_localize> / a_init_localize{},
            "home"_s <= "localize_box"_s + event<e_init_home> / a_go_home{},

            //Soft_stop() section
            "home"_s <= "making_initial_imgs"_s + event<e_init_home> / a_go_home{},
            "home"_s <= "approaching"_s + event<e_init_home> / a_go_home{},
            "home"_s <= "recover_approaching"_s + event<e_init_home> / a_go_home{},
            "home"_s <= "recover_grasping"_s + event<e_init_home> / a_go_home{},


            "making_initial_imgs"_s <= "home"_s + event<e_init_cycle> / a_making_initial_imgs{},
            "approaching"_s <= "making_initial_imgs"_s + event<e_start_approaching> / a_start_approaching{},
            "putting"_s <= "making_initial_imgs"_s + event<e_start_putting> / a_start_putting{},

            "recover_approaching"_s <= "approaching"_s + event<e_approach_fail> / a_recover_approaching{},
            "approaching"_s <= "recover_approaching"_s + event<e_start_approaching> / a_start_approaching{},
            "making_initial_imgs"_s <= "recover_approaching"_s + event<e_init_cycle> / a_making_initial_imgs{},
            "grasping"_s <= "approaching"_s + event<e_start_grasping> / a_start_grasping{} ,

            "recover_grasping"_s <= "grasping"_s + event<e_grasp_fail> / a_recover_grasping{} ,
            "grasping"_s <= "recover_grasping"_s + event<e_start_grasping> / a_start_grasping{} ,
            "recover_approaching"_s <= "recover_grasping"_s + event<e_approach_fail> / a_recover_approaching{} ,
            "putting"_s <= "grasping"_s + event<e_start_putting> / a_start_putting{} ,

            "making_initial_imgs"_s <= "putting"_s + event<e_init_cycle> / a_making_initial_imgs{},
            "approaching"_s <= "putting"_s + event<e_start_approaching> / a_start_approaching{},
            "home"_s <= "putting"_s + event<e_init_home> / a_go_home{} // automatically go home after putting
        // clang-format on
    );
  }
};

using SM_T = sml::sm<robot_world, sml::logger<robot_logger>>;
class ApplyEvent : public boost::static_visitor<> {
public:
  SM_T &sm_;
  ApplyEvent(SM_T &sm) : sm_{sm} {}
  template <class E> void operator()(E e) const { sm_.process_event(e); }
};

struct Controller : public ControllerInterface {
  std::shared_ptr<RealRobot> ri_;
  std::shared_ptr<SkodaPublisher> sp_;
  // std::shared_ptr<BehaviorInterface> bi_;
  CAdapter ca_;
  robot_logger logger_;
  SM_T sm_;
  ApplyEvent ae_;

  Controller(std::shared_ptr<RealRobot> ri, std::shared_ptr<SkodaPublisher> sp)
      : ri_{ri}, sp_(sp), ca_{this}, logger_{}, sm_{logger_, ca_}, ae_{sm_} {}

  // remove if compiles
  // void start() {
  //   emit(e_init_home{});
  //   emit(e_init_cycle{});
  // }
  void start_manual(RobotCommand cmd) {
    switch (cmd) {
    case RobotCommand::home:
      emit(e_init_home{});
      break;
    case RobotCommand::make_data:
      ri_->a_do_manual(cmd);
      break;
    case RobotCommand::grasping:
    INFO("GRASPING");
      emit(e_init_localize{});
      emit(e_init_home{});
      //emit(e_init_cycle{});
      break;
    case RobotCommand::init:
    INFO("INIT");
      g_init = true;
      emit(e_init_localize{});
      emit(e_init_home{});
      //emit(e_init_cycle{});
      break;
    default:
      break;
    }
  }

  void emit(AllEventsVariant event) { boost::apply_visitor(ae_, event); }
  virtual std::shared_ptr<SkodaPublisher> get_skoda_publisher() { return sp_; };
  std::shared_ptr<RealRobot> get_robot_interface() { return ri_; }
};

auto get_robot_client_interface(bool pn) {
  INFO("pn ", pn);
  return pn ? std::make_shared<imr::HybridRobotClient>(
                  std::make_shared<imr::RobotClient>())
            : std::make_shared<imr::HybridRobotClient>(
                  std::make_shared<imr::FakeRobotClient>());
}

/////////////////// move to another file

ignition::transport::Node::Publisher pub_error;
ignition::transport::Node::Publisher pub_status;
ignition::transport::Node::Publisher pub_info;

int process_comand(int cmd) {
  if (cmd > g_command) {
    return cmd;
  }
  return g_command;
}

auto getCommandMsgCallback(Controller &c) {
  return [&c](const CommandMsg &msg) {
    INFO("RECIEVED SIGNAL FROM GUI:");
    std::cerr << "Command: " << msg.command() << std::endl;
    std::cerr << "msg: " << msg.msg() << std::endl;
    g_command = process_comand(msg.command());
    std::cerr << "Current global command: " << g_command << std::endl
              << std::endl;
  };
}

/////////////////// move to another file

void heartbeat() {
    skoda_msgs::msgs::StatusMsg status_msg;
    status_msg.set_id(StatusMsg::BEHAVE);
    status_msg.set_status(1);

  while (true) {
    if (!pub_status.Publish(status_msg)) {
      std::cout << "FAILED TO PUBLISH HEARTBEAT" << std::endl;
      // return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void behave(RobotCommand cmd, bool pn) {

  // publish_status();
  // publish_stuff();
  std::string CamLogDir{"cam_log"};
  int CamLogIdx{-1};


  // real camera
  gige_camera::Camera gige_cam;

  auto cameraLogger = Camera::Logger(CamLogDir, &CamLogIdx);

  auto camera = std::make_shared<Camera>(gige_cam, cameraLogger);


  // virtual camera
  // auto cameraLogger = VirtualCamera::Logger(&CamLogIdx);
  // auto camera = std::make_shared<VirtualCamera>(
  // std::vector<std::string>{"test_img.jpg"}, cameraLogger);

  // robot clientskoda_msgs
  std::cout << "GET CLIENT INTERFACE" << std::endl;
  auto robot_client = get_robot_client_interface(pn);

  std::cout << "MAKE SHARED ROBOT" << std::endl;
  auto robot = std::make_shared<RealRobot>(robot_client, camera);
  auto sp = std::make_shared<SkodaPublisher>();
  Controller c(robot, sp);

  // c.start();
  ignition::transport::Node node;

  const std::string command_topic_ = "/command";
  std::function<void(CommandMsg const &)> cb = getCommandMsgCallback(c);
  if (!node.Subscribe(command_topic_, cb)) {
    std::cerr << "Error subscribing to topic [" << command_topic_ << "]"
              << std::endl;
  }
  std::cout << "Subscribed to topic [" << command_topic_ << "]" << std::endl;

  pub_status = node.Advertise<skoda_msgs::msgs::StatusMsg>("/status");
  if (!pub_status) {
    std::cerr << "Error advertising topic [/status]" << std::endl;
  }
  std::cout << "Publishing to topic [/status]" << std::endl;

  RosoutCheck rosout_check;

  std::thread t1(heartbeat);
  c.start_manual(cmd);

  t1.join();

  ignition::transport::waitForShutdown();
  
}




////////////////////////////////////////////////////////////////////////
//TEST DETECTION
////////////////////////////////////////////////////////////////////////

RobotPose toRobotPose2(YAML::Node const &n) {
  RobotPose robot_pose;
  robot_pose.x = n[0].as<double>();
  robot_pose.y = n[1].as<double>();
  robot_pose.z = n[2].as<double>();
  robot_pose.rx = n[3].as<double>();
  robot_pose.ry = n[4].as<double>();
  robot_pose.rz = n[5].as<double>();
  INFO("toRobotPose2: ok");
  return robot_pose;
}

RobotData readRobotData2(std::string const &robot_data_fname) {
  RobotData robot_data;
  auto robot_data_node = YAML::LoadFile(robot_data_fname);
  auto common_node = robot_data_node["common"];
  INFO("get common_node: ok");
  auto robot_on_object_pose_node = common_node["robot_on_object_pose"];
  INFO("get robot_on_object_pose_node: ok");
  auto camera_f = common_node["camera_f"];
  INFO("get camera_f: ok");
  auto camera_offset_z = common_node["camera_offset_z"];
  INFO("get camera_offset_z: ok");
  auto list_node = robot_data_node["list"];
  INFO("get list_node: ok");

  RobotData::Common common;
  common.robot_on_object_pose = toRobotPose2(robot_on_object_pose_node);
  common.camera_f = camera_f.as<double>();
  INFO("get camera_f: ok");
  common.camera_offset_z = camera_offset_z.as<double>();
  INFO("get camera_offset_z: ok");
  robot_data.common = common;

  RobotData::List list;
  std::transform(list_node.begin(), list_node.end(), std::back_inserter(list),
                 [](YAML::Node const &element_node) {
                   RobotData::ListElement element;
                   element.robot_pose = toRobotPose2(element_node["robot_pose"]);
                   element.camera_img_fname =
                       element_node["camera_img_fname"].as<std::string>();
                   return element;
                 });
  robot_data.list = list;
  return robot_data;
}

int area2(cv::Rect const &box) { return box.width * box.height; }

bool area_cmp2(cv::Rect const &a, cv::Rect const &b) {
  return area2(a) < area2(b);
}


void test_detection(){

  bool test_data = true;

  int idx;
  HOGDetectorLogger l("output", &idx);
  HOGDetector d(l);
  ModelClient m(5556);
  ModelClient m_occl(5557);


  if(test_data){
    int idx2 = 0;
    int incorrect = 0;
    RobotData robot_data = readRobotData2("test_detection/robot_data.yaml");

    RobotPose item_pose = robot_data.common.robot_on_object_pose;

    auto &list = robot_data.list;
    for (auto &item : list) {

      if (idx2 % 20 == 0) INFO("idx: ", idx2, "/", list.size());

      cv::Mat img = cv::imread("test_detection/"+item.camera_img_fname);

      RobotPose current_pose = *item.robot_pose;
      GeometricTransformation robot_tfm = toGeometricTransformation(current_pose);

      auto boxes = d.detect_boxes_test(img, 0.5);

      auto measurement = std::make_shared<PoseImgMeasurement>(robot_tfm, img);
      auto objs = SHIT::getObjectInferences(boxes, m, m_occl, measurement);
      auto target = SHIT::select_highest(objs);

      GeometricTransformation object_tfm = target.base_object_tfm;

      double z_diff = fabs(item_pose.z - object_tfm(2, 3));
      if( z_diff > 0.05){
        ROS_INFO("INCORRECT DETECTION IN IMAGE z_diff = %f", z_diff);
        INFO(item.camera_img_fname);
        incorrect++;
      }


      auto target_box_idx = std::distance(
          boxes.begin(), std::max_element(boxes.begin(), boxes.end(), area_cmp2));
      l.add_img_pose_info_test(robot_data.common, item, target_box_idx,
                          boxes[target_box_idx]);


      idx2++;
    }
    l.dump_info();

    ROS_INFO("Number of incorrectly detected items: %d", incorrect);
  }
  else {

    for(int i = 0; i <100; i++){

      cv::Mat img = cv::imread("test_detection/misc_imgs/00"+ std::to_string(i) + ".jpg");
      if(img.empty()) continue;

      INFO("idx: ", i);
      auto boxes = d.detect_boxes_test(img, 0.5);


    }
  }


  std::cout << "BEHAVE end" << std::endl;
}

