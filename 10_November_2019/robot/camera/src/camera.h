#ifndef CAMERA_H
#define CAMERA_H

#include "gige_camera.h"

class AbstractCamera {
public:
  virtual cv::Mat get_image() = 0;
  virtual void set_exposure_time(double us) = 0;
};

class Camera : public AbstractCamera {
private:
  double exposure_time_ = 600000;
  gige_camera::Camera &cam_;
  cv::Mat get_image_correct_orientation();
  cv::Mat element_wise_max(std::vector<cv::Mat> imgs);
  cv::Mat get_image_multiple_photo();
  void set_exposure_time(double time_us);

public:
  mutable struct Logger {
    std::string dir;
    int *num;
    Logger(std::string const &dir, int *num) : dir(dir), num(num) {}
  } log_;

  Camera(gige_camera::Camera &cam, Logger const &log) : cam_(cam), log_(log) {}

  cv::Mat get_image();
};

class VirtualCamera : public AbstractCamera {
public:
  struct Logger {
    int *img_cnt_;
    Logger(int *img_cnt) : img_cnt_(img_cnt) {}
  };

private:
  // std::string fname_;
  std::vector<std::string> img_paths_;
  Logger logger_;

public:
  cv::Mat get_image() {
    auto imidx = *(logger_.img_cnt_) % img_paths_.size();
    auto impath = img_paths_[imidx];
    *(logger_.img_cnt_) += 1;
    std::cout << "VIRTUAL CAMERA: GETTING IMAGE " << impath << std::endl;
    auto img = cv::imread(impath);
    std::cout << "VIRTUAL CAMERA: is_empty? " << img.empty() << std::endl;
    return img;
  }
  void set_exposure_time(double time_us) {}

  VirtualCamera(std::vector<std::string> const &img_paths, Logger &logger)
      : img_paths_(img_paths), logger_(logger) {}
  ~VirtualCamera() {}
};

#endif /* ifndef CAMERA_H */
