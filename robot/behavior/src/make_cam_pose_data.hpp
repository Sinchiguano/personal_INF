#ifndef MAKE_CAM_POSE_DATA_HPP
#define MAKE_CAM_POSE_DATA_HPP

#include <random>

#include "convert_3d.hpp"
#include "real_robot.hpp"
#include "utilities.hpp"

namespace PoseImgDataGather {

struct Range3D {
  Point3D a_, b_;
  Range3D(Point3D const &a, Point3D const &b) : a_(a), b_(b) {}
};

struct UniformRealDistribution3D {
  Range3D range_;
  std::uniform_real_distribution<> dis_x_;
  std::uniform_real_distribution<> dis_y_;
  std::uniform_real_distribution<> dis_z_;

  RandGenerator rg_;
  UniformRealDistribution3D(Range3D const &range)
      : range_(range), rg_{}, dis_x_(range_.a_(0), range_.b_(0)),
        dis_y_(range_.a_(1), range_.b_(1)), dis_z_(range_.a_(2), range_.b_(2)) {
  }
  Point3D next();
};

struct Item {
  cv::Mat img;
  GeometricTransformation robot_pose;
};

struct GatherStream {
  GeometricTransformation on_object_;
  double dist_;
  UniformRealDistribution3D rand_in_range_;

  GatherStream(GeometricTransformation const &on_object, double dist,
               Range3D const &range)
      : on_object_(on_object), dist_(dist), rand_in_range_{range} {}

  GeometricTransformation generatePose(RobotPose const &current);
};

void make_data(RealRobot &r);
void do_picking(RealRobot &r);
void localize_box(RealRobot &r);
} // namespace PoseImgDataGather

namespace occlusion_data {
void make_occlusion_data(RealRobot &r);
} // namespace occlusion_data

#endif
