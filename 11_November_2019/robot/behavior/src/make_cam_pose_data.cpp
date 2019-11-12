
#include <yaml-cpp/yaml.h>

#include "convert_3d.hpp"
#include "hog_detector.hpp"
#include "img_utilities.hpp"
#include "make_cam_pose_data.hpp"
#include "model_client.hpp"
#include "real_robot.hpp"
#include "utilities.hpp"
#include "data_logger.hpp"

#include "sonar_listener.hpp"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"

namespace PoseImgDataGather {

Point3D UniformRealDistribution3D::next() {
  Point3D p;
  p << dis_x_(rg_.gen_), dis_y_(rg_.gen_), dis_z_(rg_.gen_);
  INFO("random point\n", p);
  return p;
}

GeometricTransformation GatherStream::generatePose(RobotPose const &current) {
  GeometricTransformation above = on_object_ * translation(Axis3D::z, -dist_);
  GeometricTransformation rand_above =
      above * translation(rand_in_range_.next());

  // Vector3D p2 = dehom(on_object_ * Vector4D(0, 0, 0, 1));
  // Vector3D p1 = dehom(rand_above * Vector4D(0, 0, 0, 1));
  // return get_robot_pose_in_box(p1, p2, current);

  return fix_shoulder_in_box_and_clamp(get_translation(rand_above),
                                       get_translation(on_object_), current);
}

void lift_tcp(RealRobot &r, double d) {
  auto pm = r.rc_.get_pose();
  pm.z += d;
  r.rc_.set_pose(pm);
  RobotPose robot_on_object_pose = toRobotPose(r.rc_.get_pose());
  INFO("robot_on_object pose:\n", robot_on_object_pose.to_string());
}

RobotPose get_current_pose_and_backup(RealRobot &r) {
  auto pm = r.rc_.get_pose();
  RobotPose robot_on_object_pose = toRobotPose(pm);
  INFO("robot_on_object pose:\n", robot_on_object_pose.to_string());

  for (auto d : std::vector<double>{0.12}) {
    lift_tcp(r, d);
    delay(500);
  }

  return robot_on_object_pose;
}

void set_common(RobotData &robot_data, RobotPose const &robot_on_object_pose) {
  robot_data.common.robot_on_object_pose = robot_on_object_pose;
  robot_data.common.camera_f = 3820.52496;
  robot_data.common.camera_offset_z = 0.08325;
}

void make_data_old(RealRobot &r) {

  RobotPose robot_on_object_pose = get_current_pose_and_backup(r);
  RobotData robot_data;
  set_common(robot_data, robot_on_object_pose);
  CamPoseDataBase logger(
      robot_data,
      CamPoseDataPaths(CamPoseDataPaths::random_root_in("outputs")));

  RandGenerator rand_gen;
  std::uniform_int_distribution<> rand_light_low(0, 10);

  double avg_dist_for_photo = 0.25;
  double range = 0.045;
  Range3D r3d{Point3D(-range, -range, -range), Point3D(range, range, range)};

  GatherStream s(toGeometricTransformation(robot_on_object_pose),
                 avg_dist_for_photo, r3d);
  int i_img = 0;
  for (int i = 0; i < 100; i++) {
    RobotPose currentRobotPose = toRobotPose(r.rc_.get_pose());
    GeometricTransformation pose = s.generatePose(currentRobotPose);

    int move_result;
    bool centered = false;
    if(centered){
    PoseMsg not_centered = toPoseMsg(toRobotPose(pose));
    double random_scale = 60;
    double rand_change = double(rand() % 10 - 5) / random_scale;
    not_centered.a += rand_change;
    rand_change = double(rand() % 10 - 5) / random_scale;
    not_centered.b += rand_change;
    rand_change = double(rand() % 10 - 5) / random_scale;
    not_centered.c += rand_change;
    move_result = r.rc_.set_pose(not_centered);
    }
    else{
    move_result = r.rc_.set_pose(toPoseMsg(toRobotPose(pose)));
    }

    INFO("move result ", move_result);
    if (move_result != 1)
      continue;

    delay(150);
    // Three randomly generated light intensities at intervals: 0-10, 10-20,
    // 20-30
    // int light_val = rand_light_low(rand_gen.gen_);

    // NOTE sometimes we need this loop
    // for (int j = light_val; j <= 30; j += 10, i_img++) {
    // int light_val = j;

    INFO("Getting image");
    r.rc_.set_light(4);
    auto img = r.camera_->get_image();
    r.rc_.set_light(0);

    INFO("Actual robot pose:");
    auto actual_pose = r.rc_.get_pose();
    logger.push(PoseImgMeasurement(
        toGeometricTransformation(toRobotPose(actual_pose)), img));

    robot_data.list.push_back(toListElement(i_img, toRobotPose(actual_pose)));
    //   break;
    // }
  }
  std::ofstream output_str("output/robot_data.yaml");
  output_str << toYAML(robot_data);
}

/* Returns euclidean distance between two RobotPoses. */
double computeDiff(RobotPose robot_on_object_pose, RobotPose robot_pose) {
  double diffx = robot_pose.x - robot_on_object_pose.x;
  double diffy = robot_pose.y - robot_on_object_pose.y;
  double diffz = robot_pose.z - robot_on_object_pose.z;

  double ret = sqrt(diffx*diffx + diffy*diffy + diffz*diffz);

  return ret;

}

void make_data(RealRobot &r) {
  bool approaching = true;

  /*int argc;
  char ** argv;
  ros::init(argc, argv, "sonar_lis_node");*/

  SonarListener sl;

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("sonar", 1, &SonarListener::SonarCB, &sl);

  ros::Rate loop_rate(1);
  int16_t msg;
  loop_rate.sleep();

  ros::spinOnce();
  msg = sl.getMsg();
  //std::cout << "returning msg " << msg << std::endl;

  //std::ofstream of ("sonar_output.txt", std::ofstream::out);
  std::ofstream of;
  of.open("outputs_lab/exp15000/cesarTest.txt");

  of << "sonar robot_pose_diff measure\n";

  RobotPose robot_on_object_pose = get_current_pose_and_backup(r);
  RobotData robot_data1;
  set_common(robot_data1, robot_on_object_pose);

  CamPoseDataBase logger1(robot_data1,CamPoseDataPaths("outputs_lab/exp15000/cesarTest"));

  double avg_dist_for_photo = 0.25;
  double range = 0.045;
  Range3D r3d{Point3D(-range, -range, -range), Point3D(range, range, range)};

  GatherStream s(toGeometricTransformation(robot_on_object_pose),
                 avg_dist_for_photo, r3d);
  int i_img = 0;
  for (int i = 0; i < 50; i++) {

    INFO("ITERATION ", i);
    std::cout << "ITERATION: %i" <<i<< std::endl;

    RobotPose currentRobotPose = toRobotPose(r.rc_.get_pose());
    GeometricTransformation pose = s.generatePose(currentRobotPose);

    int move_result;
    bool not_centered = false;
    if(not_centered){
      PoseMsg not_centered = toPoseMsg(toRobotPose(pose));
      double random_scale = 30;
      double rand_change = double(rand() % 10 - 5) / random_scale;
      not_centered.a += rand_change;
      rand_change = double(rand() % 10 - 5) / random_scale;
      not_centered.b += rand_change;
      rand_change = double(rand() % 10 - 5) / random_scale;
      not_centered.c += rand_change;
      move_result = r.rc_.set_pose(not_centered);
    }
    else{
      move_result = r.rc_.set_pose(toPoseMsg(toRobotPose(pose)));
    }

    double diff_dist;

    //number of pictures taken
    for (int w = 0; w < 1; w++){

      INFO("move result ", move_result);
      if (move_result != 1)
        continue;

      INFO("part ", w);

      // NOTE sometimes we need this loop
      // for (int j = light_val; j <= 30; j += 10, i_img++) {
      // int light_val = j;


      r.rc_.set_light(6);
      //INFO("Set Exp ");
      r.camera_->set_exposure_time(30000);
      //INFO("Get Img ");
      auto img1 = r.camera_->get_image();
      r.rc_.set_light(0);
      delay(1000);

      //INFO("Got Img ");
      auto actual_pose = r.rc_.get_pose();

      ros::spinOnce();
      msg = sl.getMsg();
      std::cout << "returning msg " << msg << std::endl;

      logger1.push3(PoseImgMeasurement(
          toGeometricTransformation(toRobotPose(actual_pose)), img1), w, int(msg));
      delay(50);

      //diff_dist = computeDiff(robot_on_object_pose, toRobotPose(actual_pose));
      //std::cout << "distance from object acc. to robot " << diff_dist << std::endl;

      //of << msg << " " << diff_dist  << " " << 0 << std::endl;


      if(approaching){
        //Try movement iterations
        RobotPose current_pose = toRobotPose(actual_pose);
        GeometricTransformation robot_tfm = toGeometricTransformation(current_pose);

        auto measurement = std::make_shared<PoseImgMeasurement>(robot_tfm, img1);
        auto boxes = r.d.detect_boxes(img1, 0.5);

        if (boxes.size() == 0) {
          break;
        }

        auto objs = SHIT::getObjectInferences(boxes, r.m, r.m_occl, measurement);
        auto target = SHIT::select_highest(objs);

        GeometricTransformation object_tfm = target.base_object_tfm;

        //TODO: Should we change the above object position to be
        // in avg_dist_for_photo distance above the object
        // instead of fixed 0.22 m????
        GeometricTransformation above_object =
            object_tfm * translation(Vector3D(0, 0, -0.25));


        above_object = fix_shoulder_in_box_and_clamp(get_translation(above_object),
                                                    get_translation(object_tfm),
                                                    current_pose);

        if(w < 2) move_result = r.rc_.set_pose(toPoseMsg(toRobotPose(above_object)));

        //INFO("Got Img ");
        auto actual_pose = r.rc_.get_pose();

        ros::spinOnce();
        if(!sl.getNewDataBool()) ros::spinOnce();
        msg = sl.getMsg();

        std::cout << "returning msg " << msg << std::endl;

        diff_dist = computeDiff(robot_on_object_pose, toRobotPose(actual_pose));
        std::cout << "distance from object acc. to robot " << diff_dist << std::endl;
        of << " " << msg << " " << diff_dist << " " << actual_pose.z - robot_on_object_pose.z << " " << w+1 << std::endl;
      }
    }


  }

  INFO("Make_data finished");

  of.close();

  //std::ofstream output_str("output/robot_data.yaml");
  //output_str << toYAML(robot_data1);
  //std::ofstream output_str2("output/robot_data2.yaml");
  //output_str2 << toYAML(robot_data2);
  //std::ofstream output_str3("output/robot_data3.yaml");
  //output_str3 << toYAML(robot_data3);
  //std::ofstream output_str4("output/robot_data4.yaml");
  //output_str4 << toYAML(robot_data4);
}

template <class T, class F> struct SelectBest {
  T best;
  F f_;
  SelectBest(T const &t, F f) : best(t), f_{f} {}
  void operator()(T const &x) { best = std::max(best, x, f_); }
};

class HeightCmp {
  HeightCmp(RobotPose const &rp, ModelClient const &m);
  bool operator()(cv::Rect const &a, cv::Rect const &b);
  RobotPose const &rp_;
  ModelClient const &m_;
};

bool HeightCmp::operator()(cv::Rect const &a, cv::Rect const &b) {}

HeightCmp::HeightCmp(RobotPose const &rp, ModelClient const &m)
    : rp_(rp), m_(m) {}

std::vector<GeometricTransformation>
hack_lift_center_poses(std::vector<GeometricTransformation> poses) {
  // sort y grow
  std::sort(poses.begin(), poses.end(),
            [](auto const &a, auto const &b) { return a(1, 3) < b(1, 3); });
  // fix [0, 1]
  poses[0](1, 3) += 0.01;
  poses[1](1, 3) += 0.01;
  return poses;
}

// ObjectInferences getObjectInferences(auto const &boxes, auto &pose_model,
//                                     auto &occl_model,
//                                     auto const &measurement) {
//  ObjectInferences result;
//  std::transform(boxes.begin(), boxes.end(), std::back_inserter(result),
//                 [&pose_model, &occl_model, &measurement](auto const &box) {
//                   return ObjectInference(pose_model, occl_model, measurement,
//                                          box);
//                 });
//  return result;
//}

void select_first_item(RealRobot &r, HOGDetector &d, ModelClient &pose_model,
                       ModelClient &occl_model) {

  double centerX = 0.77;
  double centerY = 0.1;

  /*
  //if box is centered to x=0.77, y =0.0
  // PoseMsg left1(0.75, 0.27, 0.80, 0, 3.14, 0);
  // PoseMsg middle1(0.75, 0.0, 0.80, 0, 3.14, 0);
  // PoseMsg right1(0.75, -0.31, 0.80, 0.0, 3.14, 0.0);
  std::vector<double> left = {-0.082477, -1.237941, 0.893815,
                              -2.797400, 0.082399,  3.140052};
  std::vector<double> middle = {-0.433223, -1.214931, 0.861894,
                                -2.788416, 0.433125,  3.139944};
  std::vector<double> right = {-0.752276, -0.894550, 0.352308,
                               -2.599377, 0.752425,  3.139944};
  */

  // if box is centered to x=0.77, y =0.1
  // PoseMsg left1(0.76, 0.37, 0.85, 0, 3.14, 0);
  // PoseMsg middle1(0.76, 0.10, 0.85, 0, 3.14, 0);
  // PoseMsg right1(0.76, -0.23, 0.85, 0.0, 3.14, 0.0);
  std::vector<double> left = {0.052369,  -1.121275, 0.624970,
                              -2.645356, -0.052178, 3.139992};
  std::vector<double> middle = {-0.303691, -1.178612, 0.710534,
                                -2.673617, 0.303737,  3.139908};
  std::vector<double> right = {-0.674065, -0.881064, 0.220151,
                               -2.480726, 0.674031,  3.139908};

  delay(100);

  // Get first item
  r.rc_.set_joints_pose(left);

  cv::Mat img0, img1, img2;
  RobotPose c_pose0, c_pose1, c_pose2;
  if (ros::ok()) {
    r.rc_.set_light(4);
    img0 = r.camera_->get_image();
    r.rc_.set_light(0);
    c_pose0 = toRobotPose(r.rc_.get_pose());

    r.rc_.set_joints_pose(middle);
    r.rc_.set_light(4);
    img1 = r.camera_->get_image();
    r.rc_.set_light(0);
    c_pose1 = toRobotPose(r.rc_.get_pose());

    r.rc_.set_joints_pose(right);
    r.rc_.set_light(4);
    img2 = r.camera_->get_image();
    r.rc_.set_light(0);
    c_pose2 = toRobotPose(r.rc_.get_pose());
  }

  auto getObjectInferencesImgPose = [&pose_model, &occl_model,
                                     &d](auto img, auto const &pose) {
    GeometricTransformation robot_tfm_init = toGeometricTransformation(pose);
    auto measurement =
        std::make_shared<PoseImgMeasurement>(robot_tfm_init, img);
    auto boxes = d.detect_boxes(img, 0.5);
    auto objs =
        SHIT::getObjectInferences(boxes, pose_model, occl_model, measurement);
    return objs;
  };

  auto concat_abc = [](auto const &a, auto const &b, auto const &c) {
    auto result = a;
    std::copy(b.begin(), b.end(), std::back_inserter(result));
    std::copy(c.begin(), c.end(), std::back_inserter(result));
    return result;
  };

  // auto selllll = [&pose_model, &occl_model, &d,
  //                 getObjectInferencesImgPose](auto img, auto const &pose) {
  //   auto objs = getObjectInferencesImgPose(img, pose);
  //   ObjectInference target_object = select_highest(objs);
  //   return target_object.base_object_tfm;
  // };

  // GeometricTransformation object_tfm0 = selllll(img0, c_pose0),
  //                         object_tfm1 = selllll(img1, c_pose1),
  //                         object_tfm2 = selllll(img2, c_pose2);

  auto objs0 = getObjectInferencesImgPose(img0, c_pose0);
  auto objs1 = getObjectInferencesImgPose(img1, c_pose1);
  auto objs2 = getObjectInferencesImgPose(img2, c_pose2);
  auto objs = concat_abc(objs0, objs1, objs2);

  // auto max_z = [](auto const &object_tfm0, auto const &object_tfm1,
  //                 auto const &object_tfm2) {
  //   GeometricTransformation first_object_tfm = object_tfm0;
  //   if (first_object_tfm(2, 3) < object_tfm1(2, 3))
  //     first_object_tfm = object_tfm1;
  //   if (first_object_tfm(2, 3) < object_tfm2(2, 3))
  //     first_object_tfm = object_tfm2;
  //   return first_object_tfm;
  // };

  auto max_z_tfm = [](auto const &objects) {
    auto obj = *std::max_element(
        objects.begin(), objects.end(), [](auto const &a, auto const &b) {
          return a.base_object_tfm(2, 3) < b.base_object_tfm(2, 3);
        });
    return obj.base_object_tfm;
  };

  GeometricTransformation first_object_tfm = max_z_tfm(objs);
  // max_z(object_tfm0, object_tfm1, object_tfm2);

  GeometricTransformation above_object_init =
      first_object_tfm * translation(Vector3D(0, 0, -0.35));

  RobotPose current_pose_init = toRobotPose(r.rc_.get_pose());
  above_object_init = fix_shoulder_in_box_and_clamp(
      get_translation(above_object_init), get_translation(first_object_tfm),
      current_pose_init);

  PoseMsg ppp = toPoseMsg(toRobotPose(first_object_tfm));
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
    r.rc_.set_joints_pose(left_box_init_init);
  } else {
    r.rc_.set_joints_pose(right_box_init_init);
  }

  INFO("Go above first item: ");
  r.rc_.set_pose(toPoseMsg(toRobotPose(above_object_init)));

  delay(100);

  // finish the selection of first
}

void pick_up_object(RealRobot &r, GeometricTransformation const &final_tfm) {

  // Set obstacle window around the item on its Z level
  PoseMsg item_level =
      toPoseMsg(toRobotPose(final_tfm * translation(Vector3D(0, 0, -0.02))));
  r.rc_.set_item_level(item_level, true);

  // Pick up the part
  r.rc_.set_pose(toPoseMsg(toRobotPose(final_tfm)));
  INFO("Force mode on");
  //r.rc_.set_force_mode(true);
  INFO("Squeeze the item");
  r.rc_.set_pose(
      toPoseMsg(toRobotPose(final_tfm * translation(Vector3D(0, 0, 0.05)))));
  INFO("Vaccuum on");
  r.rc_.set_vacuum(true);
  delay(300);
  INFO("Move out");
  r.rc_.set_pose(
      toPoseMsg(toRobotPose(final_tfm * translation(Vector3D(0, 0, -0.05)))));
  INFO("Force mode off");
  //r.rc_.set_force_mode(false);

  // Remove the obstacle window
  r.rc_.set_item_level(item_level, false);

/*
  // WARNING: Still some problem with the sensors
  std::vector<URVacuumStatus> vacuum = r.rc_.vacuum_status();
  INFO("Vacuum pneuAttached: ", (int)vacuum.at(0).structured.pneuAttached, " ",
       (int)vacuum.at(1).structured.pneuAttached, " ",
       (int)vacuum.at(2).structured.pneuAttached, " ",
       (int)vacuum.at(3).structured.pneuAttached, " ",
       (int)vacuum.at(4).structured.pneuAttached, " ",
       (int)vacuum.at(5).structured.pneuAttached);
  int vac_success = 0;
  for (auto const &vac : vacuum)
    vac_success += (int)vac.structured.pneuAttached;
  INFO("Attached suction pads: ", vac_success);
  if (vac_success < 4) {
    // Turn off vacuum and start item localization from the top
    INFO("Less than 4 suction pads attached");
  } else {
    // r.rc_.attach_item();
  }
*/
  r.rc_.attach_item();

  // Move out of the box - middle steps
  std::vector<double> joint_pose = r.rc_.get_joints();
  // move up
  // If Conf 1
  if (joint_pose.at(0) > -1 && joint_pose.at(0) < 1) {
    joint_pose.at(1) = -1.5;
    joint_pose.at(2) = 1.5;
    r.rc_.set_joints_pose(joint_pose);
  }
  // else conf2
  else {
    joint_pose.at(1) = -1.5;
    joint_pose.at(2) = -1.5;
    r.rc_.set_joints_pose(joint_pose);
  }
}

void drop_item(RealRobot &r, HOGDetector &d, ModelClient &m) {

  // std::vector<double> top_line_check = {-1.654743, -1.105592, 0.025411,
  // -2.314095, -1.634577, 3.074856};
  std::vector<double> top_line_check_left = {-1.608092, -0.787464, -0.064376,
                                             -2.236440, -1.469215, 3.080161};
  std::vector<double> top_line_check_right = {-1.880270, -0.801693, -0.151437,
                                              -2.262740, -1.250615, 3.12557};

  std::vector<double> top_left_up = {-1.668852, -1.286041, 1.043559,
                                     -2.528632, -1.201926, 3.108682};
  std::vector<double> top_left_down = {-1.668852, -1.286041, 1.113559,
                                       -2.528632, -1.201926, 3.108682};
  std::vector<double> top_right_up = {-1.945183, -1.314101, 1.061343,
                                      -2.505650, -1.205630, 2.992620};
  std::vector<double> top_right_down = {-1.945183, -1.314101, 1.131343,
                                        -2.505650, -1.205630, 2.992620};

  // std::vector<double> left_box_init = {-0.238946, -1.247606, 1.474864,
  // -0.228640, -0.238796, 0.000252};  std::vector<double>
  // left_box_init_back_step = {-0.238946, -1.247606, 0.90, -2.50,
  // -1.20, 3.0};

  std::vector<double> left_box_init = {-0.201471, -1.265172, 1.362403,
                                       -0.098234, -0.149990, -0.000719};
  std::vector<double> left_box_init_back_step = {-0.201471, -1.265172, 0.90,
                                                 -2.50,     -1.20,     3.0};

  // Placing the part in the feeder

  // Adjust to the position
  // r.rc_.set_joints_pose(top_line_check);

  // Temporary
  //(go back to previous version, if we can see both left and right)
  while (true) {

    // LEFT
    r.rc_.set_joints_pose(top_line_check_left);
    // Check if lines are free
    r.rc_.set_light(4);
    auto img = r.camera_->get_image();
    r.rc_.set_light(0);

    auto boxes = d.detect_boxes(img, 0.5);
    RobotPose obstacle;
    bool full = false;

    GeometricTransformation robot_tfm =
        toGeometricTransformation(toRobotPose(r.rc_.get_pose()));

    for (int target_box = 0; target_box < boxes.size(); target_box++) {
      GeometricTransformation object_tfm =
          get_object_tfm_wrt_base(img, boxes[target_box], m, robot_tfm);
      obstacle = toRobotPose(object_tfm);
      if (obstacle.z > 0.55) { // top line
        full = true;
      }
    }

    if (!full) {
      // r.rc_.set_joints_pose(top_left_up);
      // r.rc_.set_force_mode(true);
      // r.rc_.set_joints_pose(top_left_down);
      // r.rc_.set_force_mode(false);
      r.rc_.set_vacuum(false);
      r.rc_.detach_item(true);
      // r.rc_.set_joints_pose(top_left_up);
      break;
    }

    full = false;
    // RIGHT
    r.rc_.set_joints_pose(top_line_check_right);
    r.rc_.set_light(4);
    img = r.camera_->get_image();
    r.rc_.set_light(0);

    boxes = d.detect_boxes(img, 0.5);

    robot_tfm = toGeometricTransformation(toRobotPose(r.rc_.get_pose()));

    for (int target_box = 0; target_box < boxes.size(); target_box++) {
      GeometricTransformation object_tfm =
          get_object_tfm_wrt_base(img, boxes[target_box], m, robot_tfm);
      obstacle = toRobotPose(object_tfm);
      if (obstacle.z > 0.55) { // top line
        full = true;
      }
    }

    if (!full) {
      // r.rc_.set_joints_pose(top_right_up);
      // r.rc_.set_force_mode(true);
      // r.rc_.set_joints_pose(top_right_down);
      // r.rc_.set_force_mode(false);
      r.rc_.set_vacuum(false);
      r.rc_.detach_item(true);
      // r.rc_.set_joints_pose(top_right_up);
      break;
    }

    // Wait
    delay(1000);
  }
}

void do_picking(RealRobot &r) {
  // HOG
  int idx = 0;
  HOGDetectorLogger l("output", &idx);
  HOGDetector d(l);
  ModelClient m(5556);
  ModelClient m_occl(5557);

  bool has_to_do_init_step = true;

  // sort n parts
  for (int w = 0; w < 30; w++) {

    GeometricTransformation final_tfm;
    // Picking iterations

    select_first_item(r, d, m, m_occl);

    for (int i = 0; i < 5; ++i) {
      INFO("Picking iteration");
      cv::Mat img;
      if (ros::ok()) {
        r.rc_.set_light(4);
        img = r.camera_->get_image();
        r.rc_.set_light(0);
      }
      RobotPose current_pose = toRobotPose(r.rc_.get_pose());
      GeometricTransformation robot_tfm =
          toGeometricTransformation(current_pose);

      auto measurement = std::make_shared<PoseImgMeasurement>(robot_tfm, img);
      auto boxes = d.detect_boxes(img, 0.5);
      auto objs = SHIT::getObjectInferences(boxes, m, m_occl, measurement);
      auto target = SHIT::select_highest(objs);

      GeometricTransformation object_tfm = target.base_object_tfm;
      final_tfm = object_tfm * translation(Vector3D(0, 0, -0.03));
      GeometricTransformation above_object =
          object_tfm * translation(Vector3D(0, 0, -0.25));
      above_object = fix_shoulder_in_box_and_clamp(
          get_translation(above_object), get_translation(object_tfm),
          current_pose);

      r.rc_.set_pose(toPoseMsg(toRobotPose(above_object)));
    }

    pick_up_object(r, final_tfm);

    drop_item(r, d, m);
  }
}

void localize_box(RealRobot &r) {

  // r.rc_.initPlanningScene();
}

} // namespace PoseImgDataGather

namespace occlusion_data {

GeometricTransformation
random_pose(GeometricTransformation const &initial_pose) {
  return initial_pose;
}

int uniform_random_int(int a, int b) {
  static RandGenerator rand_gen;
  std::uniform_int_distribution<> distr(a, b);
  return distr(rand_gen.gen_);
}

GeometricTransformation
closer_to_detection(cv::Mat img, cv::Rect box, ModelClient &m,
                    GeometricTransformation const &robot_tfm) {
  GeometricTransformation object_tfm =
      get_object_tfm_wrt_base(img, box, m, robot_tfm);
  // TODO randomize
  return object_tfm * translation(Vector3D(0, 0, -0.45));
}

void make_occlusion_data(RealRobot &r) {
  int idx = 0;
  HOGDetectorLogger l("output", &idx);
  HOGDetector d(l);
  ModelClient m(5556);

  auto initial_pose = toGeometricTransformation(toRobotPose(r.rc_.get_pose()));

  for (int i = 0; i < 50; ++i) {
    auto robot_pose = random_pose(initial_pose);
    r.rc_.set_pose(toRobotPose(robot_pose));
    robot_pose = toGeometricTransformation(toRobotPose(r.rc_.get_pose()));
    r.rc_.set_light(4);
    auto img = r.camera_->get_image();
    r.rc_.set_light(0);

    auto boxes = d.detect_boxes(img, 0.5);
    auto random_box = boxes[uniform_random_int(0, boxes.size() - 1)];

    auto closer_pose = closer_to_detection(img, random_box, m, robot_pose);
    r.rc_.set_pose(toRobotPose(closer_pose));
    r.rc_.set_light(4);
    img = r.camera_->get_image();
    r.rc_.set_light(0);
  }
}

} // namespace occlusion_data
