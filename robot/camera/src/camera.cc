#include <thread>
#include <iomanip>

//#include "fileutils.h"
#include "camera.h"
#include "gige_camera.h"

// using namespace imr;

cv::Mat Camera::get_image_correct_orientation() {
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  cv::Mat img = cam_.getImage();
  cv::transpose(img, img);
  return img;
}

cv::Mat Camera::element_wise_max(std::vector<cv::Mat> imgs) {
  cv::Mat ret = imgs[0].clone();
  for (int i = 0; i < imgs[0].rows; ++i) {
    for (int j = 0; j < imgs[0].cols; ++j) {
      auto max = imgs[0].at<uchar>(i, j);
      for (int k = 1; k < imgs.size(); ++k) {
        max = std::max(max, imgs[k].at<uchar>(i, j));
      }
      ret.at<uchar>(i, j) = max;
    }
  }
  return ret;
}

cv::Mat Camera::get_image_multiple_photo() {
  std::vector<cv::Mat> imgs;
  for (int i = 0; i < 3; ++i) {
    imgs.push_back(get_image_correct_orientation());
  }
  return element_wise_max(imgs);
}

cv::Mat Camera::get_image() {
  ++*log_.num;

  cv::Mat img = get_image_correct_orientation();
  // cv::Mat img = get_image_multiple_photo();

  // TODO make utility
  // TODO create dir
  std::string output_dir ("camera_imgs_log/");
  std::stringstream ss;
  //ss << output_dir << std::setfill('0') << std::setw(4) << *log_.num << ".jpg";
  ss << output_dir << "0000.jpg";
  //auto fname = ss.str();
  cv::imwrite(ss.str(), img);

  return img;
}

void Camera::set_exposure_time(double time_us) {
  cam_.set_exposure_time(time_us);
}
