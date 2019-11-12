#ifndef HOG_DETECTOR_HPP
#define HOG_DETECTOR_HPP

#include <string>

#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "convert_3d.hpp"

struct RobotData {
  struct Common {
    RobotPose robot_on_object_pose;
    double camera_f;
    double camera_offset_z;
    int distance;
  };
  struct ListElement {
    boost::optional<RobotPose> robot_pose;
    std::string camera_img_fname;
    int distance;
  };
  using List = std::vector<ListElement>;
  Common common;
  List list;
};

RobotData::ListElement toListElement(int id, RobotPose const &robot_pose);
RobotData::ListElement toListElement2(int id, int part, RobotPose const &robot_pose);
RobotData::ListElement toListElement3(int id, int part, RobotPose const &robot_pose, int distance);

YAML::Node toYAML(RobotData const &robot_data);

struct HOGDetectorLogger {
  HOGDetectorLogger(std::string const &output_dir, int *idx)
      : output_dir(output_dir), idx(idx) {
    info["list"] = YAML::Node();
    info["common"] = YAML::Node();
    debug_idx=0;
  }
  std::string output_dir;
  YAML::Node info;
  int *idx;
  int debug_idx;
  void debug_save_img(cv::Mat img);
  void draw_boxes_test(cv::Mat img, std::vector<cv::Rect> const &boxes);
  void draw_boxes_no_bg2(cv::Mat img, std::vector<cv::Rect> const &boxes);

  void draw_boxes(cv::Mat img, std::vector<cv::Rect> const &boxes);


  void draw_boxes_no_bg(cv::Mat img, std::vector<cv::Rect> const &boxes);
  void draw_bigger_boxes_no_bg(cv::Mat img, std::vector<cv::Rect> const &boxes);
  void draw_middle_boxes(cv::Mat img,
                         std::vector<cv::RotatedRect> const &boxes);
  void print_boxes_pos(std::vector<cv::Rect> const &boxes);

  void add_img_pose_info(RobotData::Common const &common,
                         RobotData::ListElement const &item,
                         int target_box_idx,
                         cv::Rect const &box);

  void add_img_pose_info_test(RobotData::Common const &common,
                         RobotData::ListElement const &item,
                         int target_box_idx,
                         cv::Rect const &box);
  void dump_info();
};


//the only thing is that i modify the constructor and i created a new function member winStride
//the HOGDetector class
//be cesar sinchiguano
class HOGDetector {
public:
  HOGDetector(HOGDetectorLogger const &logger);

  std::vector<cv::Rect> detect_boxes(cv::Mat test_img, double scale_hog_img);
  //my trick je je je
  void cesar_bypass(cv::Mat img,std::vector<cv::Rect> const &boxes);


  std::vector<cv::Rect> detect_boxes_test(cv::Mat test_img, double scale_hog_img);

  // hog parameters
  double hitThreshold;
  cv::HOGDescriptor hog;
  std::string output_dir;

  mutable HOGDetectorLogger logger_;
};

#endif
