#ifndef REAL_ROBOT_HPP
#define REAL_ROBOT_HPP

#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <netdb.h>
#include <string.h>
#include <unistd.h>

#include <zmq.hpp>
#include <boost/optional.hpp>

#include "convert_3d.hpp"
#include "hog_detector.hpp"
#include "image_publisher.hpp"
#include "model_client.hpp"
#include "robot_client.h"
#include "robot_interface.hpp"
#include "robot_measurement.hpp"
#include "box_pose.hpp"


#include "error_catch.hpp"

#include "camera.h"

struct RobotConstants {
  //
};

// TODO replace with RobotClient
// interacts with NUC by sending commands via socket.
struct RobotController {
  std::shared_ptr<imr::RobotClientInterface> client_;
  RobotController(std::shared_ptr<imr::RobotClientInterface> client)
      : client_(client) {}

  int set_pose(PoseMsg const &pose_msg);
  int set_pose(RobotPose const &pose);
  int set_joints_pose(std::vector<double> joints);
  std::vector<double> get_joints();
  PoseMsg get_pose();
  std::vector<bool> vacuum_status();
  void attach_item();
  void item_vis(double x, double y,double z);
  void detach_item(bool b);
  void set_item_level(PoseMsg pose, bool b);
  void set_vacuum(bool b);
  void set_light(int l);
  void set_force_mode(int i);
};

using PoseImgMeasurements = std::vector<PoseImgMeasurement>;
struct RealRobot {
  RobotController rc_;

  std::shared_ptr<AbstractCamera> camera_;

  int idx;
  HOGDetectorLogger l;
  HOGDetector d;
  ModelClient m;
  ModelClient m_occl;

  boost::optional<BoxPose> box_pose;

  ErrorPublisher timed_error_publisher_;

  RealRobot(std::shared_ptr<imr::RobotClientInterface> client,
            std::shared_ptr<AbstractCamera> camera)
      : rc_(client),
        camera_(camera), idx{0}, l{"output", &idx}, d{l}, m{5556},
        m_occl{5557}, timed_error_publisher_{} {}

  bool init_safe_move();
  bool init_check_suction();
  void camera_test();
  bool a_go_home();
  bool localize_box();
  bool a_do_manual(RobotCommand cmd);
  std::vector<GeometricTransformation> a_making_initial_imgs();
  //struct a_start_approaching {
    //bool success;
    //GeometricTransformation tfm;
  bool a_start_approaching(std::vector<GeometricTransformation> highest_objects,
                           GeometricTransformation &tfm);
  //}
  bool a_recover_approaching();
  bool a_recover_grasping();
  bool a_start_grasping(GeometricTransformation const &tfm);
  int a_start_putting();

private:
  cv::Mat get_light_img(int light);
  std::vector<GeometricTransformation>
  select_highest(PoseImgMeasurements const &measurements);
  void set_to_init_position(GeometricTransformation const &highest);
  PoseMsg avoid_colliding(GeometricTransformation const &desired_tfm);
  bool pick_up_object(GeometricTransformation const &final_tfm);
  void move_out_of_the_box();
  int drop_item(int position_in_feeder, bool feeder_full);
  void no_space_in_feeder();
  int detect_free_feeder_position(bool &feeder_full);
  bool init_safe_move_by_parts();
  bool init_safe_move_monolythic();

};

#endif
