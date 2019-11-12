#ifndef CONVERT_3D_HPP
#define CONVERT_3D_HPP

#include <sstream>
#include <Eigen/Dense>

using Matrix3 = Eigen::Matrix3d;
using Vector3D = Eigen::Vector3d;
using Vector4D = Eigen::Vector4d;
using Point3D = Eigen::Vector3d;
Point3D toPoint3D(double x, double y, double z);
Vector3D toVector3D(double x, double y, double z);
Vector4D toVector4D(double x, double y, double z, double w);

using ScaledAxisAngle = Eigen::Vector3d;
using Quaternion = Eigen::Vector4d;

// TODO what if angle is 0?
struct AxisAngle {
  double angle;
  Eigen::Vector3d axis;
  AxisAngle(ScaledAxisAngle const &aa) : angle(aa.norm()), axis(aa / angle) {}
  AxisAngle(Vector3D const &axis, double angle) : angle(angle), axis(axis) {}
};

ScaledAxisAngle toScaledAxisAngle(AxisAngle const &aa);

using RotationMatrix = Eigen::Matrix3d;
// TODO add functions for transformation
using GeometricTransformation = Eigen::Matrix4d;

struct RobotPose {
  double x, y, z;
  double rx, ry, rz;
  RobotPose() {}
  RobotPose(double x, double y, double z, double rx, double ry, double rz)
    :x(x), y(y), z(z), rx(rx), ry(ry), rz(rz)
  {}
  std::string to_string() const {
    std::stringstream ss;
    ss
      << "x " << x
      << ", y " << y
      << ", z " << z
      << ", rx " << rx
      << ", ry " << ry
      << ", rz " << rz;
    return ss.str();
  }
};

enum class Axis3D { x, y, z };

Quaternion toQuaternion(ScaledAxisAngle const &saa);
Quaternion toQuaternion(AxisAngle const &aa);

AxisAngle toAxisAngle(Quaternion const &q);
ScaledAxisAngle toScaledAxisAngle(Quaternion const &q);

RobotPose toRobotPose(GeometricTransformation const &T);

Vector3D get_translation(GeometricTransformation const &A);
GeometricTransformation translation(Point3D const &p);
GeometricTransformation translation(Axis3D axis, double dist);
GeometricTransformation rotation(Vector3D axis, double angle);
GeometricTransformation rotation(RotationMatrix const &R);

GeometricTransformation toGeometricTransformation(RobotPose const &rp);

RotationMatrix aa2R(AxisAngle const &a);
AxisAngle R2aa(RotationMatrix const &m);

Vector3D dehom(Vector4D const &p);
Vector4D hom(Vector3D const &p);

GeometricTransformation homogeneneous_inverse(GeometricTransformation const &A);
Vector3D get_object_axis_wrt_camera(GeometricTransformation const &cam, GeometricTransformation const &object);
Vector3D get_object_translation_wrt_camera (GeometricTransformation const &T_br, GeometricTransformation const &T_bo);

RotationMatrix get_rotation(Vector3D x_orientation, Vector3D z);
RotationMatrix get_rotation_y(Vector3D y_orientation, Vector3D z);
GeometricTransformation fix_orientation(Vector3D const &on_object,
                                        Vector3D const &robot_pos,
                                        Vector3D const &x_orientation);
GeometricTransformation fix_orientation_y(Vector3D const &on_object,
                                        Vector3D const &robot_pos,
                                        Vector3D const &x_orientation);

GeometricTransformation fix_shoulder_in_box_and_clamp(Vector3D p1, Vector3D const &p2, RobotPose const &current);

enum class Config {c00, c01, c10, c11};
Vector3D get_p3_for_config(double z, Config config);
Config get_configuration(Vector3D goal_pose, RobotPose current_pose);
GeometricTransformation get_robot_pose_in_box(Vector3D const &p1, Vector3D const &p2, RobotPose const &current);

class PoseMsg;
class RobotPose;
RobotPose toRobotPose(PoseMsg const &pm);
PoseMsg toPoseMsg(RobotPose const &p);

#endif
