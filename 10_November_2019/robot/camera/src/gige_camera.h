#ifndef GIGE_CAMERA_HPP
#define GIGE_CAMERA_HPP

#include "_Common/Common.h"
//#include "Common.h"
#include <opencv2/opencv.hpp>

namespace gige_camera {

class Camera {
public:
  Camera();
  ~Camera();
  cv::Mat getImage();
  void set_exposure_time(double val);

private:
  smcs::ICameraAPI gigeApi;
  smcs::IDevice device;

  cv::Mat copyToImg(const smcs::IImageBitmapInterface src);
};

} // namespace gige_camera

#endif /* ifndef GIGE_CAMERA_HPP */
