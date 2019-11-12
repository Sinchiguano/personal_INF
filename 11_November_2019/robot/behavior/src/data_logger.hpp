#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include <memory>
#include <random>
#include <vector>

#include "io.hpp"

// #include "PoseImgMeasurement.hpp"

// struct Node {
//   boost::Any element;
//   std::function<void(boost::Any const &)> push;
// };

/*
 */

struct AlphaDist {
  std::random_device rd;
  std::mt19937 mt;
  std::uniform_int_distribution<int> dist;
  AlphaDist() : rd{}, mt(rd()), dist('a', 'z') {}
};

auto RandomStringGenerator(int size) {
  return [ size, d = std::make_unique<AlphaDist>() ]() mutable {
    std::string result;
    std::generate_n(std::back_inserter(result), size,
                    [&] { return d->dist(d->mt); });
    return result;
  };
}

struct CamPoseDataPaths {
  std::string root_;
  std::string yaml_dump_fname_;

  CamPoseDataPaths(std::string const &root)
      : root_(root), yaml_dump_fname_(root_ + "/robot_data.yaml") {
    make_dirs();
  }

  void dump_yaml(YAML::Node const &n) {
    std::ofstream output_str(yaml_dump_fname_);
    output_str << n;
  }

  void save_img(cv::Mat img, int counter) {
    std::stringstream ss;
    ss << root_ << "/camera_imgs/" << std::setfill('0') << std::setw(4)
       << counter << ".jpg";
    auto fname = ss.str();
    cv::imwrite(ss.str(), img);
  }

  void save_img2(cv::Mat img, int counter, int part) {
    std::stringstream ss;
    ss << root_ << "/camera_imgs/" << std::setfill('0') << std::setw(4)
       << counter << "_" << std::to_string(part) << ".jpg";
    auto fname = ss.str();
    cv::imwrite(ss.str(), img);
  }

  void make_dirs() { ::create_dir(root_ + "/camera_imgs"); }

  static std::string random_root_in(std::string const &dirname) {
    auto gen = RandomStringGenerator(15);
    return dirname + "/" + gen() + "/"; // TODO check if exists
  }
};

struct CamPoseDataBase {
  int counter_;
  RobotData data_;
  CamPoseDataPaths paths_;

  CamPoseDataBase(RobotData const &data, CamPoseDataPaths const &paths)
      : counter_{0}, data_(data), paths_(paths) {}

  ~CamPoseDataBase() { paths_.dump_yaml(toYAML(data_)); }

  void push(PoseImgMeasurement const &measurement) {
    paths_.save_img(measurement.img, counter_);
    data_.list.push_back(
        toListElement(counter_, toRobotPose(measurement.robot_pose)));
    ++counter_;
  }

  void push2(PoseImgMeasurement const &measurement, int part) {
    if(part == 0) ++counter_;
    paths_.save_img2(measurement.img, counter_, part);
    data_.list.push_back(
        toListElement2(counter_, part, toRobotPose(measurement.robot_pose)));
  }

  void push3(PoseImgMeasurement const &measurement, int part, int distance) {
    if(part == 0) ++counter_;
    paths_.save_img2(measurement.img, counter_, part);
    data_.list.push_back(
        toListElement3(counter_, part, toRobotPose(measurement.robot_pose), distance));
  }

};

/*

API
{
    auto db = make_database()
    with:
        data(robot_on_object)
        destructor = dump_yaml
        push = ...

    db.push(PoseImgMeasurement(img, pose));
}
*/

#endif // !DATA_LOGGER_HPP