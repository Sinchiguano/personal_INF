
#include "robot_measurement.hpp"
#include "model_client.hpp"

PoseImgMeasurement::PoseImgMeasurement(GeometricTransformation const &rp,
                                       cv::Mat const &im)
    : robot_pose{rp}, img{im} {}

ObjectInference::ObjectInference(
    ModelClient &pose_model, ModelClient &occl_model,
    std::shared_ptr<PoseImgMeasurement> measurement, cv::Rect const &box)
    : measurement{measurement}, box{box},
      base_object_tfm{get_object_tfm_wrt_base(measurement->img, box, pose_model,
                                              measurement->robot_pose)},
      occl_pred{get_occlusion_via_server(measurement->img, box, occl_model)} {}