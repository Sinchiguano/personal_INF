#include <iostream>
#include <random>

#include "app_log.hpp"
#include "convert_3d.hpp"
#include "hog_detector.hpp"
#include "make_cam_pose_data.hpp"
#include "random_failure.hpp"
#include "real_robot.hpp"
#include "robot_measurement.hpp"
#include "utilities.hpp"

#include "log_publisher.hpp"

std::vector<int> init_vacuum_values;

PoseMsg RobotController::get_pose() { return client_->get_pose(); }

std::vector<bool> RobotController::vacuum_status() {
  return client_->vacuum_status();
}

int RobotController::set_pose(PoseMsg const &pose_msg) {
  return client_->set_pose(pose_msg);
}

int RobotController::set_pose(RobotPose const &pose) {
  return client_->set_pose(toPoseMsg(pose));
}

int RobotController::set_joints_pose(std::vector<double> joints) {
  // INFO("RobotController::set_joints_pose");
  return client_->set_joints_pose(joints);
}

std::vector<double> RobotController::get_joints() {
  // INFO("RobotController::get_joints");
  return client_->get_joints();
}

void RobotController::attach_item() {
  INFO("RobotController::attach_item");
  client_->attach_item();
}
void RobotController::item_vis(double x, double y, double z) {
  INFO("RobotController::item_vis");
  client_->item_vis(x, y, z);
}
void RobotController::detach_item(bool b) {
  INFO("RobotController::detach_item");
  client_->detach_item(b);
}
void RobotController::set_item_level(PoseMsg pose, bool b) {
  INFO("RobotController::set_item_level");
  client_->set_item_level(pose, b);
}
void RobotController::set_vacuum(bool b) {
  INFO("RobotController::set_vacuum");
  client_->set_vacuum(b);
}
void RobotController::set_light(int l) {
  //INFO("RobotController::set_light");
  client_->set_light(l);
}

void RobotController::set_force_mode(int i) { client_->set_force_mode(i); }

// TMP
// RobotData::ListElement toListElement(int id, RobotPose const &robot_pose) {
//   std::stringstream ss;
//   ss << "camera_imgs/" << std::setw(4) << std::setfill('0') << id << ".jpg";
//   RobotData::ListElement elem;
//   elem.robot_pose = robot_pose;
//   elem.camera_img_fname = ss.str();
//   return elem;
// }

// TODO important: need to remove PoseMsg and use RobotPose instead
// (dream on)

void write_pose(RealRobot &r) {
  auto pose = r.rc_.get_pose();
  INFO("Robot Pose: ", pose.to_string());
}

RobotPose ran_pose_2(RobotPose const &current_pose) {
  double box_x_min = 0.60,
         box_x_max = 0.93, // 95
      box_y_min = -0.05,   // Make it as big as possible
      box_y_max = 0.37;

  // generate p1, p2
  DistributionGenerator<std::uniform_real_distribution<>> p2x_gen(
      std::uniform_real_distribution<>(box_x_min, box_x_max));
  DistributionGenerator<std::uniform_real_distribution<>> p2y_gen(
      std::uniform_real_distribution<>(box_y_min, box_y_max));
  DistributionGenerator<std::uniform_real_distribution<>> p2z_gen(
      std::uniform_real_distribution<>(-0.3, -0.15));
  Vector3D p2(p2x_gen(), p2y_gen(), p2z_gen());
  DistributionGenerator<std::uniform_real_distribution<>> pos_drift(
      std::uniform_real_distribution<>(-0.025, 0.025));
  DistributionGenerator<std::uniform_real_distribution<>> pos_drift_z(
      std::uniform_real_distribution<>(0.05, 0.20));

  double z_drift = pos_drift_z();
  double drift_rate = z_drift / 0.20;
  Vector3D p1(clamp(p2(0) + pos_drift() * drift_rate, box_x_min, box_x_max),
              clamp(p2(1) + pos_drift() * drift_rate, box_y_min, box_y_max),
              p2(2) + z_drift);

  return toRobotPose(get_robot_pose_in_box(p1, p2, current_pose));
}

bool RealRobot::a_do_manual(RobotCommand cmd) {
  switch (cmd) {
  case RobotCommand::make_data:
    PoseImgDataGather::make_data(*this);
    break;
  case RobotCommand::make_occlusion_data:
    occlusion_data::make_occlusion_data(*this);
    break;
  case RobotCommand::grasping:
    PoseImgDataGather::localize_box(*this);
    PoseImgDataGather::do_picking(*this);
    break;
  default: // RobotCommand::debug
    PoseImgDataGather::localize_box(*this);
    // test_move2(*this);
    break;
  }
  return true;
}

bool RealRobot::a_go_home() {
  std::cout << "going to home..." << std::endl;
  std::vector<double> home = {0.052369,  -1.121275, 0.624970,
                              -2.645356, -0.052178, 3.135992};
  rc_.set_joints_pose(home);
  // auto img = camera_->get_image();
  return true;
}

bool RealRobot::init_safe_move_monolythic() {
  PoseMsg init_pose = rc_.get_pose();
  init_pose.z = init_pose.z + 0.2;
  if (rc_.set_pose(init_pose) > 0) {
    std::cout << "Robot at dangerously low level, going up 0.02 m" << std::endl;
    return true;
  }
  return false;
  //safe
}

bool RealRobot::init_safe_move_by_parts() {
  PoseMsg init_pose = rc_.get_pose();
  bool ret = false;
  for (int i = 0; i < 10; i++) {
    std::cout << "Robot at dangerously low level, going up 0.02 m" << std::endl;
    init_pose.z = init_pose.z + 0.02;
    ret = rc_.set_pose(init_pose);
  }
  return ret;
}

void RealRobot::camera_test() {


  std::vector<double> left_box_init_init = {-0.201471, -1.265172, 1.362403,
                                            -0.098234, -0.149990, -0.000719};
  std::vector<double> look_at_me = {-0.201471, -1.265172, 1.162403,
                                            -1.3234, -0.049990, -0.000719};
  //rc_.set_joints_pose(left_box_init_init);
  PoseMsg pose(0.75, 0.30, 0.20, 0, 3.14, 0);
  rc_.set_pose(pose);

  for(int i = 0; i < 2; i++){
    get_light_img(4);
    delay(3000);
  }

  rc_.set_joints_pose(look_at_me);

}

bool RealRobot::init_safe_move() {

/*
for(int i = 0; i < 50; i++){
  //TODO - uncomment code, when vacuum status works
  int vac_success = 0;
  std::vector<bool> vacuum = rc_.vacuum_status();
  for (auto const &vac : vacuum){
    vac_success += (int)vac;
    INFO("Vacuum: ", (int)vac);
  }
  INFO("Number of attached suction pads: ", vac_success);
  delay(2000);
}

delay(50000);
*/

  //std::vector<double> home = {0.052369,  -1.121275, 0.624970,
  //                            -2.645356, -0.052178, 3.135992};
  //rc_.set_joints_pose(home);
/*

  PoseMsg pose1 = rc_.get_pose();

  PoseMsg object(pose1.x, pose1.y, pose1.z, 3.14, 0, 0);

  object.z = pose1.z + 0.25;

  rc_.set_pose(object);
  delay(500);
  get_light_img(1);
  delay(10000);

  object.z = pose1.z + 0.5;
  rc_.set_pose(object);
  delay(500);
  get_light_img(4);
  delay(10000);

  object.z = pose1.z + 1;
  rc_.set_pose(object);
  delay(500);
  get_light_img(7);


  delay(50000);
*/

  //Turn off light
  rc_.set_light(0);

  PoseMsg init_pose = rc_.get_pose();
  std::cout << "INIT SAFE MOVE" << std::endl;
  bool success = true;
  if (init_pose.z < 0.05) {
    bool success_one_move = init_safe_move_monolythic();
    if (!success_one_move) {
      success =  init_safe_move_by_parts();
    }
  }

  delay(2000);
  std::cout << "INIT SAFE MOVE DONE (wait)" << std::endl;

  return success;
}

bool RealRobot::init_check_suction() {

  bool suction = true;
  INFO("Read vacuum status");
  int vac_success = 0;
//cesar sinchiguano
//   std::vector<bool> vacuum = rc_.vacuum_status();
//   for (auto const &vac : vacuum){
//     vac_success += (int)vac;
//     //INFO("Vacuum: ", (int)vac);
//   }
//   std::stringstream ss;
//   ss << "attached_suction_pads_init " << vac_success;
//
//   INFO("Number of attached suction pads: ", vac_success);
//   LoggerPub log(ss.str());
//
// //TODO REMOVE
vac_success = 6;
  if(vac_success > 0) {
    return true;
  } else {
    rc_.set_vacuum(false);
    return false;
  }
}

bool RealRobot::localize_box() {
  // Take pictures, localization is still not implemented
  std::cout << "Localize box..." << std::endl;
  std::vector<double> home = {0.052369,  -1.21275,  0.624970,
                              -2.645356, -0.052178, 3.139992};

  std::vector<double> left_box_init_init = {-0.201471, -1.265172, 1.362403,
                                            -0.098234, -0.149990, -0.000719};
  rc_.set_joints_pose(left_box_init_init);

  // PoseMsg pose1(0.50, 0.55, 0.40, 0, 3.14, 0);
  // PoseMsg pose2(1.00, 0.55, 0.40, 0, 3.14, 0);
  // PoseMsg pose3(1.00, -0.35, 0.40, 3.14, 0, 0);
  // PoseMsg pose4(0.50, -0.35, 0.40, 3.14, 0, 0);

  std::vector<double> left_front = {0.390367,  -1.512740, 1.523767,
                                    -0.011410, 0.390303,  -0.001691};
  std::vector<double> left_back = {0.211860,  -0.653862, 0.653028,
                                   -3.143621, -0.212015, 3.141980};
  std::vector<double> right_front = {-0.663632, -1.681466, 1.689384,
                                     -0.010464, 2.477992,  -0.002147};
  std::vector<double> righ_back = {-0.342412, -0.688994, 0.303132,
                                   0.380791,  2.800440,  -0.004485};

  cv::Mat img0, img1, img2, img3;
  RobotPose c_pose0, c_pose1, c_pose2, c_pose3;
  if (ros::ok()) {
    rc_.set_joints_pose(left_front);
    rc_.set_light(4);
    img0 = camera_->get_image();
    rc_.set_light(0);
    c_pose0 = toRobotPose(rc_.get_pose());

    rc_.set_joints_pose(left_back);
    rc_.set_light(4);
    img1 = camera_->get_image();
    rc_.set_light(0);
    c_pose1 = toRobotPose(rc_.get_pose());

    rc_.set_joints_pose(righ_back);
    rc_.set_light(4);
    img2 = camera_->get_image();
    rc_.set_light(0);
    c_pose2 = toRobotPose(rc_.get_pose());

    rc_.set_joints_pose(right_front);
    rc_.set_light(4);
    img3 = camera_->get_image();
    rc_.set_light(0);
    c_pose3 = toRobotPose(rc_.get_pose());

    using pim = PoseImgMeasurement;
    PoseImgMeasurements measurements{
      pim(toGeometricTransformation(c_pose0), img0),
      pim(toGeometricTransformation(c_pose1), img1),
      pim(toGeometricTransformation(c_pose2), img2),
      pim(toGeometricTransformation(c_pose3), img3)
    };
    box_pose = BoxPose(measurements);
  }

  return true;
}

std::vector<GeometricTransformation> RealRobot::a_making_initial_imgs() {
  std::cout << "making initial images..." << std::endl;

  // if box is centered to x=0.77, y =0.1
  // PoseMsg left1(0.76, 0.37, 0.85, 0, 3.14, 0);
  // PoseMsg middle1(0.76, 0.10, 0.85, 0, 3.14, 0);
  // PoseMsg right1(0.76, -0.23, 0.85, 0.0, 3.14, 0.0);
  std::vector<double> left = {0.052369,  -1.121275, 0.624970,
                              -2.645356, -0.052178, 3.135992};
  std::vector<double> middle = {-0.303691, -1.178612, 0.710534,
                                -2.673617, 0.303737,  3.135908};
  std::vector<double> right = {-0.674065, -0.881064, 0.220151,
                               -2.480726, 0.674031,  3.135908};

  delay(100);

  // Get first item
  rc_.set_joints_pose(left);

  cv::Mat img0, img1, img2;
  RobotPose c_pose0, c_pose1, c_pose2;
  camera_->set_exposure_time(8000);

  img0 = get_light_img(4);
  c_pose0 = toRobotPose(rc_.get_pose());

  rc_.set_joints_pose(middle);
  img1 = get_light_img(4);
  c_pose1 = toRobotPose(rc_.get_pose());

  rc_.set_joints_pose(right);
  img2 = get_light_img(4);
  c_pose2 = toRobotPose(rc_.get_pose());

  camera_->set_exposure_time(8000);

  PoseImgMeasurements measurements = PoseImgMeasurements{
      PoseImgMeasurement(toGeometricTransformation(c_pose0), img0),
      PoseImgMeasurement(toGeometricTransformation(c_pose1), img1),
      PoseImgMeasurement(toGeometricTransformation(c_pose2), img2)};

  return select_highest(measurements);
}

double tfm_x_y_distance(auto &object1, auto &object2) {
  double xdiff = object1(0, 3) - object2(0, 3);
  double ydiff = object1(1, 3) - object2(1, 3);
  double dist = sqrt(xdiff * xdiff + ydiff * ydiff);
  return dist;
};

std::vector<GeometricTransformation>
RealRobot::select_highest(PoseImgMeasurements const &measurements) {

  std::cout << "Select Highest" << std::endl;
  std::vector<GeometricTransformation> highest_objects;

  auto getObjectInferencesImgPose = [this](auto img, auto const &pose) {
    GeometricTransformation robot_tfm_init = pose;
    auto measurement =
        std::make_shared<PoseImgMeasurement>(robot_tfm_init, img);
    auto boxes = d.detect_boxes(img, 0.5);
    auto objs = SHIT::getObjectInferences(boxes, m, m_occl, measurement);
    return objs;
  };

  auto concat_abc = [](auto const &a, auto const &b, auto const &c) {
    auto result = a;
    std::copy(b.begin(), b.end(), std::back_inserter(result));
    std::copy(c.begin(), c.end(), std::back_inserter(result));
    return result;
  };

  auto img0 = measurements[0].img;
  auto c_pose0 = measurements[0].robot_pose;
  auto img1 = measurements[1].img;
  auto c_pose1 = measurements[1].robot_pose;
  auto img2 = measurements[2].img;
  auto c_pose2 = measurements[2].robot_pose;

  auto objs0 = getObjectInferencesImgPose(img0, c_pose0);
  auto objs1 = getObjectInferencesImgPose(img1, c_pose1);
  auto objs2 = getObjectInferencesImgPose(img2, c_pose2);
  auto objs = concat_abc(objs0, objs1, objs2);

  // No objs
  if (objs.size() == 0) {
    return highest_objects;
  }

  auto max_z_tfm2 = [](auto &objects) {
    auto obj = objects.at(0);
    int index = 0;
    for (int i = 1; i < objects.size(); i++) {
      if (objects.at(index).base_object_tfm(2, 3) <
          objects.at(i).base_object_tfm(2, 3)) {
        index = i;
      }
    }
    obj = objects.at(index);
    objects.erase(objects.begin() + index);
    return obj.base_object_tfm;
  };

  std::cout << "Get first highest" << std::endl;

  // Select highest object
  GeometricTransformation first_object_tfm = max_z_tfm2(objs);

  // max_z(object_tfm0, object_tfm1, object_tfm2);
  // highest_objects.push_back(first_object_tfm);

  highest_objects.push_back(first_object_tfm);
  std::cout << "SH - first object at: " << first_object_tfm(2, 3) << std::endl;

  std::cout << "SH - Num of boxes: " << objs.size() << std::endl;

  std::stringstream ss;
  ss << "Init_selected_objects ";

  double height_hysteresis = 0.06;

  for (int i = 0; i < 15; i++) {
    if (objs.size() > 0) {
      auto next_object = max_z_tfm2(objs);
      // remove next_object from objs
      if (next_object(2, 3) > (first_object_tfm(2, 3) - height_hysteresis)) {

        bool distance_check = true;
        for (int j = 0; j < highest_objects.size(); j++) {
          double dist = tfm_x_y_distance(highest_objects.at(j), next_object);
          if (dist < 0.08) {
            distance_check = false;
          }
          // std::cout << "SH - dist: " << dist << std::endl;
        }

        if (distance_check) {
          std::cout << "SH - next object at: " << next_object(2, 3)
                    << std::endl;
          ss << ", obj " << std::to_string(i) << " : " << toPoseMsg(toRobotPose(next_object)).to_string();
          highest_objects.push_back(next_object);
        }
      }
    }
  }

  LoggerPub log("Init_selected_objects_number " + std::to_string(highest_objects.size()));
  LoggerPub log2(ss.str());
  return highest_objects;
}

void RealRobot::set_to_init_position(GeometricTransformation const &highest) {
  double centerY = 0.1;

  GeometricTransformation above_object_init =
      highest * translation(Vector3D(0, 0, -0.35));
  RobotPose current_pose_init = toRobotPose(rc_.get_pose());

  above_object_init = fix_shoulder_in_box_and_clamp(
      get_translation(above_object_init), get_translation(highest),
      current_pose_init);

  PoseMsg ppp = toPoseMsg(toRobotPose(highest));
  INFO("First item pose: ", ppp.x, ", ", ppp.y, ", ", ppp.z, ", ", ppp.a, ", ",
       ppp.b, ", ", ppp.c);

  std::vector<double> left_box_init_init = {-0.201471, -1.265172, 1.362403,
                                            -0.098234, -0.149990, -0.000719};
  std::vector<double> right_box_init_init = {-3.035040, -1.858175, -1.353721,
                                             -3.099223, 0.072679,  0.034619};

  PoseMsg first_pose = toPoseMsg(toRobotPose(above_object_init));
  INFO("First target pose: ", first_pose.x, ", ", first_pose.y, ", ",
       first_pose.z, ", ", first_pose.a, ", ", first_pose.b, ", ",
       first_pose.c);

  // Go to init position above box - helps with planning
  if (first_pose.y > (centerY - 0.1)) {
    rc_.set_joints_pose(left_box_init_init);
  } else {
    rc_.set_joints_pose(right_box_init_init);
  }

  INFO("Go above first item: ");
  LoggerPub log("Approach 0 obj_pose: " + ppp.to_string());
  LoggerPub log2("Approach 0 go_to: " + first_pose.to_string());
  rc_.set_pose(toPoseMsg(toRobotPose(above_object_init)));
}

cv::Mat RealRobot::get_light_img(int light) {
  cv::Mat img;
  if (ros::ok()) {
    rc_.set_light(light);
    LoggerPub log("TAKE_IMAGE");
    TimeBomb bomb(5000, &timed_error_publisher_, 2);
    img = camera_->get_image();
    LoggerPub log2("IMAGE_TAKEN");
    rc_.set_light(0);
  }
  return img;
}

bool RealRobot::a_start_approaching(
    std::vector<GeometricTransformation> highest_objects,
    GeometricTransformation &tfm) {
  std::cout << "start approaching..." << std::endl;
  // bool f = maybe_failure(40);

  // auto highest_objects = select_highest(measurements);
  auto highest = highest_objects.at(0);
  set_to_init_position(highest);

  for (int i = 0; i < 3; ++i) {
    INFO("Picking iteration");
    auto img = get_light_img(4);
    RobotPose current_pose = toRobotPose(rc_.get_pose());
    GeometricTransformation robot_tfm = toGeometricTransformation(current_pose);

    auto measurement = std::make_shared<PoseImgMeasurement>(robot_tfm, img);
    auto raw_boxes = d.detect_boxes(img, 0.5);
    std::vector<cv::Rect> boxes;

    //TBD - experimental HOT_FIX
    //At this point there shouldnt be items too far away from the gripper
    //Boxes with smaller sizes are often incorrect detections at the edge of the picture
    for (int j = 0; j < raw_boxes.size(); j++){
      if(raw_boxes.at(j).width < 330 || raw_boxes.at(j).height < 330){
        std::stringstream ss3;
        ss3 <<"Potentionaly_dangerous_hog_detection width: "
          << std::to_string(raw_boxes.at(j).width)  <<" height: " << std::to_string(raw_boxes.at(j).height)
          << " pose_in_picture x, y: " << std::to_string(raw_boxes.at(j).x) << ", "<< std::to_string(raw_boxes.at(j).y);
        LoggerPub log3(ss3.str());
      }
      else {
        boxes.push_back(raw_boxes.at(j));
      }
    }



    if (boxes.size() == 0) {
      return false;
    }

    auto objs = SHIT::getObjectInferences(boxes, m, m_occl, measurement);
    auto target = SHIT::select_highest(objs);

    GeometricTransformation object_tfm = target.base_object_tfm;
    tfm = object_tfm;
    GeometricTransformation above_object =
        object_tfm * translation(Vector3D(0, 0, -0.25));
    above_object = fix_shoulder_in_box_and_clamp(get_translation(above_object),
                                                 get_translation(object_tfm),
                                                 current_pose);
    std::stringstream ss, ss2;
    ss <<"Approach " << std::to_string(i+1)  <<" obj_pose: " << toPoseMsg(toRobotPose(object_tfm)).to_string();
    ss2 <<"Approach " << std::to_string(i+1)  <<" go_to: " << toPoseMsg(toRobotPose(above_object)).to_string();
    LoggerPub log(ss.str());
    LoggerPub log2(ss2.str());
    rc_.set_pose(toPoseMsg(toRobotPose(above_object)));
  }
  return true;
}

bool RealRobot::a_recover_approaching() {
  std::cout << "recover approaching..." << std::endl;
  return true;
}
bool RealRobot::a_recover_grasping() {
  std::cout << "recover grasping..." << std::endl;
  return true;
}

void RealRobot::move_out_of_the_box() {
  // Move out of the box - middle steps
  std::vector<double> joint_pose = rc_.get_joints();
  // move up
  // If Conf 1
  if (joint_pose.at(0) > -1 && joint_pose.at(0) < 1) {
    joint_pose.at(1) = -1.5;
    joint_pose.at(2) = 1.5;
    rc_.set_joints_pose(joint_pose);
  }
  // else conf2
  else {
    joint_pose.at(1) = -1.5;
    joint_pose.at(2) = -1.5;
    rc_.set_joints_pose(joint_pose);
  }
}

PoseMsg clamp_in_box(PoseMsg p1) {
  double cilinder_width = 0.075;
  double centerX = 0.775;
  double centerY = 0.105;
  double x_width = 0.24;//0.275
  double y_width = 0.24;//0.475
  double box_x_max = centerX + x_width - cilinder_width;
  double box_y_max = centerY + y_width - cilinder_width;
  double box_x_min = centerX - x_width + cilinder_width;
  double box_y_min = centerY - y_width + cilinder_width;

  p1.x = clamp(p1.x, box_x_min, box_x_max);
  p1.y = clamp(p1.y, box_y_min, box_y_max);

  INFO("  NEW NEW NEW: clamped p1.x: ", p1.x, " p1.y:", p1.y, " p1.z:", p1.z);
  return p1;
}



bool in_range(RobotPose pose){
  double cilinder_width = 0.075;

  double centerX = 0.775;
  double centerY = 0.105;
  double x_width = 0.275;
  double y_width = 0.475;
  double max_x = centerX + x_width;
  double max_y = centerY + y_width;
  double min_x = centerX - x_width;
  double min_y = centerY - y_width;

  if(pose.x + cilinder_width > max_x ||
      pose.x - cilinder_width < min_x ||
      pose.y + cilinder_width > max_y||
      pose.y - cilinder_width < min_y)
  {
    return false;
  }

  return true;
}

PoseMsg RealRobot::avoid_colliding(GeometricTransformation const &desired_tfm) {
INFO("  NEW NEW NEW: Avoid Colliding");

//Collision defined by a cilinder with radius="0.075" length="0.190"
  RobotPose current_pose = toRobotPose(rc_.get_pose());
  //double box_wall_width = 0.04;
  double box_top = 0.075;
  //double phi = 0.0; //unused
  double cilinder_length = 0.190;

  RobotPose down_pose = toRobotPose(desired_tfm);

  //we are above the box
  if (down_pose.z > box_top) return toPoseMsg(toRobotPose(desired_tfm));

INFO("  NEW NEW NEW: Avoid Colliding2");

  double in_box = (box_top - desired_tfm(3,2));
  if(in_box > cilinder_length) in_box = cilinder_length;

  GeometricTransformation up_yours = desired_tfm * translation(Vector3D(0, 0, in_box));
  RobotPose up_pose = toRobotPose(up_yours);

  bool dp = in_range(down_pose);

  bool up = in_range(up_pose);

  if(dp && up) return toPoseMsg(toRobotPose(desired_tfm));

INFO("  NEW NEW NEW: Avoid Colliding3");

  if(!dp) INFO("  NEW NEW NEW: DP NOT IN RANGE");
  if(!up) INFO("  NEW NEW NEW: UP NOT IN RANGE");

  //State is colliding, somehow keep only the Z axis angle, IDK

  GeometricTransformation tmp_low = desired_tfm * translation(Vector3D(0, 0, 0.1));

  GeometricTransformation goto_tfm = get_robot_pose_in_box(get_translation(desired_tfm), get_translation(tmp_low), current_pose);
  return clamp_in_box(toPoseMsg(toRobotPose(goto_tfm)));
}

// GeometricTransformation another_avoid_colliding(GeometricTransformation const &desired_tfm) {
//   if (above_box(desired_tfm)) {
//     return desired_tfm;
//   }
//   p1 = clamp_p1;
//   p2 = clamp_p2;

// }

bool RealRobot::pick_up_object(GeometricTransformation const &final_tfm)
{
  double centerX = 0.775;
  double centerY = 0.105;

  std::stringstream obj;
  obj <<"Pick_up_object" << " obj_pose: " << toPoseMsg(toRobotPose(final_tfm)).to_string();
  LoggerPub log(obj.str());

  RobotPose current_pose = toRobotPose(rc_.get_pose());
  GeometricTransformation above_object =final_tfm * translation(Vector3D(0, 0, -0.25));
  above_object = fix_shoulder_in_box_and_clamp(get_translation(above_object), get_translation(final_tfm), current_pose);

  //rc_.set_pose(avoid_colliding(above_object));
  rc_.set_pose(toPoseMsg(toRobotPose(above_object)));

  // Set obstacle window around the item on its Z level
  PoseMsg item_level =toPoseMsg(toRobotPose(final_tfm * translation(Vector3D(0, 0, -0.10))));
  rc_.set_item_level(item_level, true);

  // rc_.item_vis(final_tfm(0,3), final_tfm(1,3), final_tfm(2,3));

  RobotPose item_pose = toRobotPose(final_tfm);
  INFO("Picking item at position: ", item_pose.x, " ", item_pose.y, " ",item_pose.z, " ", item_pose.rx, " ", item_pose.ry, " ", item_pose.rz);

  // Pick up the part
  current_pose = toRobotPose(rc_.get_pose());
  //testing....
  INFO("Before picking the part, the robot it is close enough and in parellel position to the part!");

  // TODO - get ideal safety distance from the item
  // 0.05 looks ok
  double safety_distance = 0.08;
  //int last_move_success = rc_.set_pose(avoid_colliding(final_tfm * translation(Vector3D(0, 0, -safety_distance))));
  int last_move_success = rc_.set_pose(toPoseMsg(toRobotPose(final_tfm * translation(Vector3D(0, 0, -safety_distance)))));
  // int last_move_success = rc_.set_pose(go_here);

  if (last_move_success < 0)
  {
    rc_.set_item_level(item_level, false);
    return false;
  }

  item_level =toPoseMsg(toRobotPose(final_tfm * translation(Vector3D(0, 0, -0.04))));
  rc_.set_item_level(item_level, true);
  INFO("close to the part");
  sleep(20);
  INFO("Force mode on");
  rc_.set_force_mode(1);
  INFO("Squeeze the item");

  //last_move_success = rc_.set_pose(
  //    avoid_colliding(final_tfm * translation(Vector3D(0, 0, 0.02))));
  last_move_success = rc_.set_pose(toPoseMsg(toRobotPose(final_tfm * translation(Vector3D(0, 0, 0.04)))));

  // last_move_success = rc_.set_pose(go_here);
  // If no proper plan is found for the last 5 cm
  if (last_move_success < 0)
  {
    rc_.set_item_level(item_level, false);
    rc_.set_force_mode(0);
    return false;
  }

  INFO("Vacuum on");
  rc_.set_vacuum(true);
  delay(300);
  INFO("Move out");


  // bool set_pose_success = rc_.set_pose(
  //     avoid_colliding(final_tfm * translation(Vector3D(0, 0, -0.08))));
  //Move a bit to the center of the box
  PoseMsg before_back_up = rc_.get_pose();
  PoseMsg back_up = toPoseMsg(toRobotPose(final_tfm * translation(Vector3D(0, 0, -safety_distance))));
  double z_diff = back_up.z - before_back_up.z;
  back_up.x += (centerX - back_up.x) * z_diff/2;
  back_up.y += (centerY - back_up.y) * z_diff/4;
  INFO("NEW Backup adjustment x: ",(centerX - back_up.x) * z_diff, ", y: ", (centerY - back_up.y) * z_diff/2);

  bool set_pose_success = rc_.set_pose(back_up);
  // bool set_pose_success = rc_.set_pose(go_here);
  PoseMsg test_move = rc_.get_pose();
  bool above_box_success = true;
  if(test_move.z < 0.075)
  {
    test_move.z = 0.075;
    //Fast force mode - normal speed, only cartesian
    INFO("Go above box");
    rc_.set_force_mode(2);
    delay(50);
    above_box_success = rc_.set_pose(test_move);
  }


  INFO("Force mode off");
  if (!(set_pose_success || above_box_success))
  {
    INFO("failed to move above the item");
  }
  rc_.set_force_mode(0);


  INFO("Read vacuum status");
  int vac_success = 0;
  //done by cesar sinchiguano
  // std::vector<bool> vacuum = rc_.vacuum_status();
  // for (auto const &vac : vacuum){
  //   vac_success += (int)vac;
  //   //INFO("Vacuum: ", (int)vac);
  // }
  // std::stringstream ss;
  // ss << "attached_suction_pads " << vac_success;
  // INFO("Number of attached suction pads: ", vac_success);
  // LoggerPub log2(ss.str());


  //TODO REMOVE
  vac_success = 6;

  // TODO: the sensors do not work properly
  if (vac_success < 2)
  { //} && false) {
    // Turn off vacuum and start item localization from the top
    INFO("Less than 2 suction pads attached");
    rc_.set_vacuum(false);

    if(!above_box_success) move_out_of_the_box();
    // Remove the obstacle window
    rc_.set_item_level(item_level, false);
    return false;
  } else {
    if(!above_box_success) move_out_of_the_box();
    // rc_.attach_item();
    // Remove the obstacle window
    rc_.set_item_level(item_level, false);
    return true;
  }

  return true;
}

bool RealRobot::a_start_grasping(GeometricTransformation const &tfm) {
  std::cout << "grasping..." << std::endl;
  bool f = pick_up_object(tfm);
  return f;
}

int RealRobot::detect_free_feeder_position(bool &feeder_full) {
  int time_limit = 40;
  //old
  //std::vector<double> top_line_check_left = {-1.608092, -0.787464, -0.064376,
  //                                           -2.236440, -1.469215, 3.080161};
  //std::vector<double> top_line_check_right = {-1.880270, -0.801693, -0.151437,
  //                                            -2.262740, -1.250615, 3.12557};

    std::vector<double> top_line_check_left = {-1.420060, -1.251490, 0.953999, -2.855365, -1.686938, 3.110047};
  std::vector<double> top_line_check_right = {-1.822898, -1.251466, 0.953807, -2.855257, -1.686974, 3.109819};

  // Detect free position in the feeder

  for (int i = 0; i < time_limit; i++) {

    // LEFT
    LoggerPub log("Detect_free_feeder LEFT");
    rc_.set_joints_pose(top_line_check_left);
    // Check if lines are free
    auto img = get_light_img(4);

    auto boxes = d.detect_boxes(img, 0.5);
    RobotPose obstacle;
    bool full = false;

    GeometricTransformation robot_tfm =
        toGeometricTransformation(toRobotPose(rc_.get_pose()));

    for (int target_box = 0; target_box < boxes.size(); target_box++) {
      GeometricTransformation object_tfm =
          get_object_tfm_wrt_base(img, boxes[target_box], m, robot_tfm);
      obstacle = toRobotPose(object_tfm);
      if (obstacle.z > 0.24) { // top line
        full = true;
      }
    }

    if (!full) {
      return 1;
    }

    full = false;
    // RIGHT
    LoggerPub log2("Detect_free_feeder RIGHT");
    rc_.set_joints_pose(top_line_check_right);
    img = get_light_img(4);

    boxes = d.detect_boxes(img, 0.5);

    robot_tfm = toGeometricTransformation(toRobotPose(rc_.get_pose()));

    for (int target_box = 0; target_box < boxes.size(); target_box++) {
      GeometricTransformation object_tfm =
          get_object_tfm_wrt_base(img, boxes[target_box], m, robot_tfm);
      obstacle = toRobotPose(object_tfm);
      if (obstacle.z > 0.24) { // top line
        full = true;
      }
    }

    if (!full) {
      return 2;
    }

  feeder_full = true;
    // Wait
    delay(15000);
  }

  return -1;
}

int RealRobot::drop_item(int position_in_feeder, bool feeder_full) {
/*
  std::vector<double> top_left_up = {-1.668852, -1.286041, 1.043559,
                                     -2.528632, -1.201926, 3.108682};
  std::vector<double> top_left_down = {-1.668852, -1.286041, 1.113559,
                                       -2.528632, -1.201926, 3.108682};
  std::vector<double> top_right_up = {-1.945183, -1.314101, 1.061343,
                                      -2.505650, -1.205630, 2.992620};
  std::vector<double> top_right_down = {-1.945183, -1.314101, 1.131343,
                                        -2.505650, -1.205630, 2.992620};
*/
    std::vector<double> top_left_up = {-1.578803, -1.249177, 1.520942, -3.111148, -0.795110, 3.016976};
  std::vector<double> top_left_down = {-1.578803, -1.249177, 1.570942, -3.111148, -0.795110, 3.016976};
  std::vector<double> top_right_up = {-1.980105, -1.234799, 1.512008, -3.105546, -0.735693, 2.956375};
  std::vector<double> top_right_down = {-1.980105, -1.234799, 1.562008, -3.105546, -0.735693, 2.956375};

  INFO("Read vacuum status on drop");
  int vac_success = 0;

  //cesar sinchiguano
  // std::vector<bool> vacuum = rc_.vacuum_status();
  // for (auto const &vac : vacuum){
  //   vac_success += (int)vac;
  //   //INFO("Vacuum: ", (int)vac);
  // }
  // std::stringstream ss;
  // ss << "attached_suction_pads_on_drop " << vac_success;
  // INFO("Number of attached suction pads on drop: ", vac_success);
  // LoggerPub log2(ss.str());
  //int success = 0;


  int success = 0;


//TODO REMOVE
vac_success = 6;

  // TODO: the sensors do not work properly
  if (vac_success > 0 && feeder_full) { //} && false) {
    success = 2;
  } else if(vac_success > 0){
    success = 1;
  }

  switch (position_in_feeder) {
  case 1: {
    rc_.set_joints_pose(top_left_up);
    rc_.set_force_mode(1);
    rc_.set_joints_pose(top_left_down);
    rc_.set_force_mode(0);
    rc_.set_vacuum(false);
    //rc_.detach_item(true);
    rc_.set_joints_pose(top_left_up);
    break;
  }
  case 2: {
    rc_.set_joints_pose(top_right_up);
    rc_.set_force_mode(1);
    rc_.set_joints_pose(top_right_down);
    rc_.set_force_mode(0);
    rc_.set_vacuum(false);
    //rc_.detach_item(true);
    rc_.set_joints_pose(top_right_up);
    break;
  }
  }

  return success;
}

void RealRobot::no_space_in_feeder(){
  std::vector<double> left_box_init_init = {-0.201471, -1.265172, 1.362403,
                                            -0.098234, -0.149990, -0.000719};

  std::vector<double> middle = {-0.303691, -1.178612, 0.710534,
                                -2.673617, 0.303737,  3.135908};
  rc_.set_joints_pose(middle);

  auto img = get_light_img(4);
  RobotPose current_pose = toRobotPose(rc_.get_pose());
  GeometricTransformation robot_tfm = toGeometricTransformation(current_pose);
  auto measurement = std::make_shared<PoseImgMeasurement>(robot_tfm, img);
  auto boxes = d.detect_boxes(img, 0.5);

  if (boxes.size() == 0) {
    rc_.set_joints_pose(left_box_init_init);
    rc_.set_vacuum(false);
    return;
  }

  auto objs = SHIT::getObjectInferences(boxes, m, m_occl, measurement);

  auto max_z_tfm2 = [](auto &objects) {
    auto obj = objects.at(0);
    int index = 0;
    for (int i = 1; i < objects.size(); i++) {
      if (objects.at(index).base_object_tfm(2, 3) <
          objects.at(i).base_object_tfm(2, 3)) {
        index = i;
      }
    }
    obj = objects.at(index);
    objects.erase(objects.begin() + index);
    return obj.base_object_tfm;
  };

  GeometricTransformation highest_obj = max_z_tfm2(objs);

    GeometricTransformation above_object =
        highest_obj * translation(Vector3D(0, 0, -0.20));
    above_object = fix_shoulder_in_box_and_clamp(get_translation(above_object),
                                                 get_translation(highest_obj),
                                                 current_pose);

    rc_.set_pose(toPoseMsg(toRobotPose(above_object)));

  rc_.set_vacuum(false);
}


int RealRobot::a_start_putting() {
  std::cout << "putting..." << std::endl;

  int item_movement_success = 0;
  bool feeder_full = false;
  //light is obstructed - higher exposition time needed
  camera_->set_exposure_time(33000);
  int position_in_feeder = detect_free_feeder_position(feeder_full);
  camera_->set_exposure_time(8000);
  if (position_in_feeder < 0 ) {
    std::cout << "No free space in the feeder for the last 2 minutes..."
              << std::endl;
    no_space_in_feeder();

    return 3;
  } else {
    LoggerPub log("Free_feeder_position: " + std::to_string(position_in_feeder) );
    item_movement_success = drop_item(position_in_feeder, feeder_full);
  }

  return item_movement_success;
}

RobotCommand parseRobotCommand(std::string const &cmd_str) {
  // clang-format off
  RobotCommand cmd = cmd_str == "make_data" ? RobotCommand::make_data
                        : cmd_str == "make_occlusion_data" ? RobotCommand::make_occlusion_data
                        : cmd_str == "grasping" ? RobotCommand::grasping
                        : cmd_str == "home" ? RobotCommand::home
                        : cmd_str == "init" ? RobotCommand::init
                        : RobotCommand::debug;
  // clang-format on
  return cmd;
}
