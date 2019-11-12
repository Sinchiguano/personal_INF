#include <fstream>
#include <iomanip>

#include <yaml-cpp/yaml.h>


#include "convert_3d.hpp"
#include "hog_detector.hpp"
#include "img_utilities.hpp"
#include "io.hpp"
#include "utilities.hpp"

#include "log_publisher.hpp"

void saveYAML(std::vector<float> const &vals, double hitThreshold,
              std::string fname)
              {
  YAML::Node n;
  n["descriptor"] = vals;
  n["hitThreshold"] = hitThreshold;
  std::ofstream ofs(fname);
  ofs << n << "\n";
}

void loadYAML(std::vector<float> &vals, double &hitThreshold,
              std::string fname) {
  try {
    YAML::Node n = YAML::LoadFile(fname);
    vals = n["descriptor"].as<std::vector<float>>();
    hitThreshold = n["hitThreshold"].as<double>();
  } catch (YAML::BadFile e) {
    std::cerr << "Error: cannot load HOG model (bad file)" << std::endl;
  }
}

RobotData::ListElement toListElement(int id, RobotPose const &robot_pose) {
  std::stringstream ss;
  ss << "camera_imgs/" << std::setw(4) << std::setfill('0') << id << ".jpg";
  RobotData::ListElement elem;
  elem.robot_pose = robot_pose;
  elem.camera_img_fname = ss.str();
  return elem;
}

RobotData::ListElement toListElement2(int id, int part, RobotPose const &robot_pose) {
  std::stringstream ss;
  ss << "camera_imgs/" << std::setw(4) << std::setfill('0') << id << "_" << std::to_string(part) << ".jpg";
  RobotData::ListElement elem;
  elem.robot_pose = robot_pose;
  elem.camera_img_fname = ss.str();
  return elem;
}

RobotData::ListElement toListElement3(int id, int part, RobotPose const &robot_pose, int distance) {
  std::stringstream ss;
  ss << "camera_imgs/" << std::setw(4) << std::setfill('0') << id << "_" << std::to_string(part) << ".jpg";
  RobotData::ListElement elem;
  elem.robot_pose = robot_pose;
  elem.camera_img_fname = ss.str();
  elem.distance = distance;
  return elem;
}


//////////////////////default setup//////////////////////////start_contructor
/*
HOGDetector::HOGDetector(HOGDetectorLogger const &logger)
    : hog(cv::Size(64, 64), cv::Size(16, 16), cv::Size(4, 4), cv::Size(8, 8), 9,
          1, -1, cv::HOGDescriptor::L2Hys, 0.2, true, 80),
      logger_(logger)
{
  bool train = false;
  std::string descriptingVectorSave("descriptingVectorSave.yaml");
  std::vector<float> descriptorVector;
  if (train) {
    // training not implemented yet
  } else {
    loadYAML(descriptorVector, hitThreshold, descriptingVectorSave);
  }
  hog.setSVMDetector(descriptorVector);

  // TODO remove?
  this->output_dir = "output";
}
*/
//////////////////////default setup//////////////////////////end


/////////////////////Friday 08 Nov 2019///////////////////////////
/////////////////////cesar trick_start///////////////////////////

//created by cesar sinchiguano, the original one is above.
HOGDetector::HOGDetector(HOGDetectorLogger const &logger)
      : logger_(logger) {
  this->output_dir = "output";
}
/////////////////////cesar trick_end/////////////////////

std::string counted_name_with_dir(std::string const &pre,
                                  std::string const &post,
                                  std::vector<int> idxs)
                                  {
  auto output_dir_boxes = pre;
  create_dir(output_dir_boxes);
  std::stringstream s;
  s << output_dir_boxes;
  std::stringstream idxs_s;
  for (auto i = idxs.cbegin(), preend = --idxs.cend(); i != idxs.cend(); ++i) {
    s << std::setw(4) << std::setfill('0') << *i;
    if (i != preend) {
      s << "_";
    }
  }
  s << post;
  return s.str();
}

std::vector<double> get_robot_to_object_tfm_descriptor(RobotData::Common const &common,RobotData::ListElement const &item)
{
  GeometricTransformation camera = toGeometricTransformation(*item.robot_pose);
  GeometricTransformation object = toGeometricTransformation(common.robot_on_object_pose);
  Vector3D axis = get_object_axis_wrt_camera(camera, object);
  Vector3D translation = get_object_translation_wrt_camera(camera, object);
  return std::vector<double>{axis(0),        axis(1),        axis(2),
                             translation(0), translation(1), translation(2)};
}

YAML::Node toYAML(cv::Rect const &box)
{
  YAML::Node n;
  n["x"] = box.x;
  n["y"] = box.y;
  n["height"] = box.height;
  n["width"] = box.width;
  return n;
}

void HOGDetectorLogger::add_img_pose_info(RobotData::Common const &common,RobotData::ListElement const &item,int target_box_idx,cv::Rect const &box)
{
  YAML::Node n;
  n["box_fname"] = counted_name_with_dir("boxes_no_bg/", ".jpg", std::vector<int>{*idx, target_box_idx});
  n["robot_to_object_tfm_descriptor"] = get_robot_to_object_tfm_descriptor(common, item);
  n["box_pos"] = toYAML(box);
  info["list"].push_back(n);
}

void HOGDetectorLogger::dump_info()
{
  std::string info_fname = output_dir + "/after_hog_data.yaml";
  std::ofstream ofs(info_fname);
  ofs << info;
}

void HOGDetectorLogger::debug_save_img(cv::Mat img)
{
  cv::Mat img_boxes = img.clone();
  auto output_dir_boxes = output_dir + "/debug_all_img/";
  create_dir(output_dir_boxes);
  std::stringstream s;
  s << output_dir_boxes << std::setw(4) << std::setfill('0') << debug_idx << ".jpg";
  cv::imwrite(s.str(), img_boxes);
  debug_idx++;
}
void HOGDetectorLogger::draw_boxes(cv::Mat img,std::vector<cv::Rect> const &boxes)
{
  cv::Mat img_boxes = img.clone();
  img_boxes = ::draw_boxes(img_boxes, boxes, cv::Scalar(0, 255, 0), 3);
  auto output_dir_boxes = output_dir + "/boxes/";
  create_dir(output_dir_boxes);
  std::stringstream s;
  s << output_dir_boxes << std::setw(4) << std::setfill('0') << *idx << ".jpg";
  cv::imwrite(s.str(), img_boxes);
}
void HOGDetectorLogger::draw_bigger_boxes_no_bg(cv::Mat img, std::vector<cv::Rect> const &boxes)
{
  int cnt{0};
  double scale = 1.5;
  std::string output_dir_boxes = output_dir + "/bigger_boxes_no_bg/";
  create_dir(output_dir_boxes);
  for (auto const &box : boxes)
  {
    auto rot_box = get_bigger_box(box, scale);
    cv::Mat cropped = crop_rotated(img, rot_box);
    std::stringstream s;
    s << std::setw(4) << std::setfill('0') << *idx << "_" << std::setw(4)<< std::setfill('0') << cnt << ".jpg";
    cv::imwrite(output_dir_boxes + s.str(), cropped);
    ++cnt;
  }
}
void HOGDetectorLogger::draw_boxes_no_bg(cv::Mat img,std::vector<cv::Rect> const &boxes)
{
  int cnt{0};
  for (auto const &box : boxes)
  {
    cv::Mat cropped(cv::Mat(img, box));
    auto output_dir_boxes = output_dir + "/boxes_no_bg/";
    create_dir(output_dir_boxes);
    std::stringstream s;
    s << std::setw(4) << std::setfill('0') << *idx << "_" << std::setw(4)
      << std::setfill('0') << cnt << ".jpg";
    cv::imwrite(output_dir_boxes + s.str(), cropped);
    ++cnt;
  }
}
std::vector<cv::Rect> detect_hog_img(const cv::HOGDescriptor &hog,const double hitThreshold,cv::Mat &imageData)
{
  std::vector<cv::Rect> found;
  cv::Size padding(cv::Size(8, 8));
  cv::Size winStride(cv::Size(8, 8));
  hog.detectMultiScale(imageData, found, hitThreshold, winStride, padding);
  return found;
}

//-----------------------------------------------------------------------

std::vector<cv::Rect> HOGDetector::detect_boxes(cv::Mat img,double scale_hog_img)
{
  LoggerPub log("HOG_DETECT_BOXES");
  cv::Mat img_hog;

  //TODO - test it
  scale_hog_img = 1;
  //i see no meaning about this... cesar sinchiguano, if you know it, let me know, please.
  cv::resize(img, img_hog, cv::Size(), scale_hog_img, scale_hog_img,cv::INTER_LINEAR);
  //std::vector<cv::Rect> detect_hog_img
  auto test_found_rect = detect_hog_img(hog, 1.0 * hitThreshold, img_hog);

  std::transform(test_found_rect.begin(), test_found_rect.end(),
                 test_found_rect.begin(), [scale_hog_img](cv::Rect r){
                                                                      r.width /= scale_hog_img;
                                                                      r.height /= scale_hog_img;
                                                                      r.x /= scale_hog_img;
                                                                      r.y /= scale_hog_img;
                                                                      return r;
                                                                    });
  std::transform(test_found_rect.begin(), test_found_rect.end(), test_found_rect.begin(),
                  [&img](cv::Rect const &rect) { return intersection(rect, img); });
  auto boxes = test_found_rect;

  //logger_.debug_save_img(img);
  logger_.draw_boxes(img, boxes);
  ////logger_.draw_bigger_boxes_no_bg(img, boxes);
  ////logger_.draw_boxes_no_bg(img, boxes);
  // logger_.print_boxes_pos(boxes);

  LoggerPub log2("HOG_BOXES_DETECTED");
  return boxes;
}

// jeje je je by casch
void HOGDetector::cesar_bypass(cv::Mat img,std::vector<cv::Rect> const &boxes)
{
  LoggerPub log("HOG_DETECT_BOXES");
  //logger_.debug_save_img(img);
  logger_.draw_boxes(img, boxes);
  logger_.draw_bigger_boxes_no_bg(img, boxes);
  logger_.draw_boxes_no_bg(img, boxes);
  // logger_.print_boxes_pos(boxes);

  LoggerPub log2("HOG_BOXES_DETECTED");
}
////////////////////////////////
//Test detection
////////////////////////////////

void HOGDetectorLogger::draw_boxes_test(cv::Mat img, std::vector<cv::Rect> const &boxes) {
  cv::Mat img_boxes = img.clone();
  img_boxes = ::draw_boxes(img_boxes, boxes, cv::Scalar(0, 255, 0), 3);
  auto output_dir_boxes = output_dir + "/test_detection/";
  create_dir(output_dir_boxes);
  std::stringstream s;
  s << output_dir_boxes << std::setw(4) << std::setfill('0') << debug_idx << ".jpg";
  cv::imwrite(s.str(), img_boxes);
  debug_idx++;
}


void HOGDetectorLogger::draw_boxes_no_bg2(cv::Mat img,
                                         std::vector<cv::Rect> const &boxes)
                                         {
  int cnt{0};
  for (auto const &box : boxes) {
    cv::Mat cropped(cv::Mat(img, box));
    auto output_dir_boxes = output_dir + "/test_detection_boxes/";
    create_dir(output_dir_boxes);
    std::stringstream s;
    s << std::setw(4) << std::setfill('0') << debug_idx << ".jpg";
    cv::imwrite(output_dir_boxes + s.str(), cropped);
    ++cnt;
  }
  //debug_idx++;
}

int d_indx = 0;

void HOGDetectorLogger::add_img_pose_info_test(RobotData::Common const &common,
                                          RobotData::ListElement const &item,
                                          int target_box_idx,
                                          cv::Rect const &box) {
  YAML::Node n;

  std::stringstream s;
  s << "test_detection_boxes/";
  s << std::setw(4) << std::setfill('0') << d_indx << ".jpg";

  n["box_fname"] = s.str();

  n["robot_to_object_tfm_descriptor"] =
      get_robot_to_object_tfm_descriptor(common, item);
  n["box_pos"] = toYAML(box);
  info["list"].push_back(n);
  d_indx++;
}


std::vector<cv::Rect> HOGDetector::detect_boxes_test(cv::Mat img,
                                                double scale_hog_img) {
  cv::Mat img_hog;
  cv::resize(img, img_hog, cv::Size(), scale_hog_img, scale_hog_img,
             cv::INTER_LINEAR);

  auto test_found_rect = detect_hog_img(hog, 1.0 * hitThreshold, img_hog);
  std::transform(test_found_rect.begin(), test_found_rect.end(),
                 test_found_rect.begin(), [scale_hog_img](cv::Rect r) {
                   r.width /= scale_hog_img;
                   r.height /= scale_hog_img;
                   r.x /= scale_hog_img;
                   r.y /= scale_hog_img;
                   return r;
                 });
  std::transform(
      test_found_rect.begin(), test_found_rect.end(), test_found_rect.begin(),
      [&img](cv::Rect const &rect) { return intersection(rect, img); });
  auto boxes = test_found_rect;

  //logger_.debug_save_img(img);
  logger_.draw_boxes_test(img, boxes);
  //logger_.draw_bigger_boxes_no_bg(img, boxes);
  logger_.draw_boxes_no_bg2(img, boxes);
  // logger_.print_boxes_pos(boxes);

  return boxes;
}
