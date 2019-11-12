#ifndef ROBOT_MEASUREMENT_HPP
#define ROBOT_MEASUREMENT_HPP

#include <memory>

#include <opencv2/opencv.hpp>

#include "convert_3d.hpp"

struct PoseImgMeasurement {
  GeometricTransformation robot_pose;
  cv::Mat img;
  PoseImgMeasurement(GeometricTransformation const &rp, cv::Mat const &im);
};
using PoseImgMeasurements = std::vector<PoseImgMeasurement>;

class ModelClient;

struct ObjectInference {
  std::shared_ptr<PoseImgMeasurement> measurement;
  cv::Rect box;
  GeometricTransformation base_object_tfm;
  std::vector<double> occl_pred;

  ObjectInference(ModelClient &pose_model, ModelClient &occl_model,
                  std::shared_ptr<PoseImgMeasurement> measurement,
                  cv::Rect const &box);
};
using ObjectInferences = std::vector<ObjectInference>;


//////////////////////////////////////////////
// TODO to cpp
namespace SHIT {
inline ObjectInferences getObjectInferences(auto const &boxes, auto &pose_model,
                                     auto &occl_model,
                                     auto const &measurement) {
  ObjectInferences result;
  std::transform(boxes.begin(), boxes.end(), std::back_inserter(result),
                 [&pose_model, &occl_model, &measurement](auto const &box) {
                   return ObjectInference(pose_model, occl_model, measurement,
                                          box);
                 });
  return result;
}


inline bool is_occluded(ObjectInference const &obj) { return obj.occl_pred[1] > 0.5; }

inline auto filter_occluded(auto const &objs) {
  std::vector<ObjectInference> result;
  for (auto const &obj : objs) {
    if (!is_occluded(obj)) {
      result.push_back(obj);
    }
  }
  return result;
}

inline ObjectInference select_highest(std::vector<ObjectInference> const &objs_) {

  auto objs = filter_occluded(objs_);
  // auto objs = objs_;
  auto target_box_iter = std::max_element(
      objs.begin(), objs.end(), [](auto const &a, auto const &b) {
        return a.base_object_tfm(2, 3) < b.base_object_tfm(2, 3);
      });
  return *target_box_iter;
}
}
//////////////////////////////////////////////

#endif // !1ROBOT_MEASUREMENT_HPP