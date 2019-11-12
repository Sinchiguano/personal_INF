#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "app_log.hpp"
#include "convert_3d.hpp"
#include "utilities.hpp"

Eigen::Matrix3d aa2R(AxisAngle const &aa) {
  AxisAngle axan(aa);
  auto angle = axan.angle;
  auto vx = axan.axis(0);
  auto vy = axan.axis(1);
  auto vz = axan.axis(2);
  Eigen::Matrix3d S;
  // clang-format off
  S <<
    0, -vz, vy,
    vz, 0, -vx,
    -vy, vx, 0;
  // clang-format on
  RotationMatrix I = Eigen::Matrix3d::Identity();
  RotationMatrix R = I + std::sin(angle) * S + (1 - std::cos(angle)) * S * S;
  return R;
}

template <class Container>
auto sqrt_sqr_norm(Container const &elements) {
  auto sum = 0.0;
  for (auto e : elements) {
    sum += e * e;
  }
  return sqrt(sum);
}

// TODO test for singularity
AxisAngle R2aa(RotationMatrix const &m) {

  auto angle = std::acos((m(0, 0) + m(1, 1) + m(2, 2) - 1.0) / 2.0);
  if (std::cos(angle) > 0.99) {
    Vector3D aa(0,0,0);
    return aa;
  }

  auto x = (m(2, 1) - m(1, 2)) *
           sqrt_sqr_norm(std::vector<double>{
               (m(2, 1) - m(1, 2)), (m(0, 2) - m(2, 0)), (m(1, 0) - m(0, 1))});

  auto y = (m(0, 2) - m(2, 0)) *
           sqrt_sqr_norm(std::vector<double>{
               (m(2, 1) - m(1, 2)), (m(0, 2) - m(2, 0)), (m(1, 0) - m(0, 1))});

  auto z = (m(1, 0) - m(0, 1)) *
           sqrt_sqr_norm(std::vector<double>{
               (m(2, 1) - m(1, 2)), (m(0, 2) - m(2, 0)), (m(1, 0) - m(0, 1))});

  Vector3D axis = toVector3D(x, y, z);

  //if (std::abs(x) < 0.01 && std::abs(y) < 0.01 && std::abs(z) < 0.01) {}
  if (axis.norm() < 0.01) {
    Matrix3 mi = m + Matrix3::Identity();
    Vector3D v0 = mi.block<3,1>(0,0);
    Vector3D v1 = mi.block<3,1>(0,1);
    Vector3D v2 = mi.block<3,1>(0,2);
    auto v0n = v0.norm();
    auto v1n = v1.norm();
    auto v2n = v2.norm();
    axis = (v0n > v1n && v0n > v2n) ? v0 : (v1n > v2n) ? v1 : v2;
  }

  axis /= axis.norm();

  return AxisAngle(axis, angle);
}

Vector4D toVector4D(double x, double y, double z, double w) {
  Vector4D p;
  p << x, y, z, w;
  return p;
}

Vector3D toVector3D(double x, double y, double z) {
  Vector3D p;
  p << x, y, z;
  return p;
}

Point3D toPoint3D(double x, double y, double z) {
  Point3D p;
  p << x, y, z;
  return p;
}

ScaledAxisAngle toScaledAxisAngle(AxisAngle const &aa) {
  Vector3D saa = aa.axis;
  saa *= aa.angle;
  return saa;
}

RobotPose toRobotPose(GeometricTransformation const &T) {
  RotationMatrix rotation = T.block<3, 3>(0, 0);
  Point3D translation = T.block<3, 1>(0, 3);

  auto rot_part = R2aa(rotation);
  ScaledAxisAngle saa = toScaledAxisAngle(rot_part);

  RobotPose rp;
  rp.x = translation(0);
  rp.y = translation(1);
  rp.z = translation(2);
  rp.rx = saa(0);
  rp.ry = saa(1);
  rp.rz = saa(2);

  return rp;
}

Quaternion toQuaternion(AxisAngle const &aa) {
  double theta_2 = aa.angle / 2;
  double c = std::cos(theta_2);
  double s = std::sin(theta_2);
  return Quaternion(c, s * aa.axis(0), s * aa.axis(1), s * aa.axis(2));
}

Quaternion toQuaternion(ScaledAxisAngle const &saa) {
  AxisAngle aa(saa);
  return toQuaternion(aa);
}

AxisAngle toAxisAngle(Quaternion const &q) {
  double s = std::sqrt(q(1)*q(1) + q(2)*q(2) + q(3)*q(3));
  double theta = 2 * std::atan2(s, q(0));
  return s > 0 ?
        AxisAngle(Vector3D(q(1) / s, q(2) / s, q(3) / s), theta) :
        AxisAngle(Vector3D(1.0, 0.0, 0.0), theta);
}

ScaledAxisAngle toScaledAxisAngle(Quaternion const &q) {
  AxisAngle aa = toAxisAngle(q);
  return toScaledAxisAngle(aa);
}

GeometricTransformation translation(Point3D const &p) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = p(0);
  T(1, 3) = p(1);
  T(2, 3) = p(2);
  return T;
}

GeometricTransformation translation(Axis3D axis, double dist) {
  Point3D t(0, 0, 0);
  t((int)axis) = dist;
  return translation(t);
}

GeometricTransformation rotation(RotationMatrix const &R) {
  GeometricTransformation T = GeometricTransformation::Identity();
  T.block<3, 3>(0, 0) = R;
  return T;
}

GeometricTransformation rotation(Vector3D axis, double angle) {
  AxisAngle aa(axis, angle);
  RotationMatrix R = aa2R(aa);
  return rotation(R);
}

GeometricTransformation toGeometricTransformation(RobotPose const &rp) {
  Point3D translation_p(rp.x, rp.y, rp.z);
  GeometricTransformation translation_ = translation(translation_p);

  GeometricTransformation rotation_ = GeometricTransformation::Identity();
  ScaledAxisAngle rotation_p(rp.rx, rp.ry, rp.rz);
  rotation_.block(0, 0, 3, 3) = aa2R(AxisAngle(rotation_p));

  return translation_ * rotation_;
}

Vector3D dehom(Vector4D const &p) {
  Vector3D dh;
  dh << p.block<3, 1>(0, 0);
  return dh;
}

Vector4D hom(Vector3D const &p) {
  Vector4D h;
  h.block<3, 1>(0, 0) = p;
  h(3) = 1;
  return h;
}

#include "real_robot.hpp"
RobotPose toRobotPose(PoseMsg const &pm) {
  RobotPose p;
  p.x = pm.x;
  p.y = pm.y;
  p.z = pm.z;
  p.rx = pm.a;
  p.ry = pm.b;
  p.rz = pm.c;
  return p;
}

PoseMsg toPoseMsg(RobotPose const &p) {
  PoseMsg pm(p.x, p.y, p.z, p.rx, p.ry, p.rz);
  return pm;
}

Vector3D get_translation(GeometricTransformation const &A) {
    return A.block(0, 3, 3, 1);
}

Vector3D get_minus_z_axis (GeometricTransformation const &T) {
    Vector4D p0 = T * (Vector4D() << 0,0,0,1).finished();
    Vector4D p1 = T * (Vector4D() << 0,0,-60,1).finished();
    Vector3D p0_nonh = dehom(p0);
    Vector3D p1_nonh = dehom(p1);
    return (p1_nonh-p0_nonh);
}

GeometricTransformation homogeneneous_inverse(GeometricTransformation const &A) {
    Eigen::Matrix3d R = A.block(0,0,3,3);
    Vector3D d = get_translation(A);
    GeometricTransformation inv {GeometricTransformation::Identity()};
    inv.block(0,0,3,3) = R.inverse();
    inv.block(0,3,3,1) = -R.inverse() * d;
    return inv;
}

Vector3D get_object_axis_wrt_camera (GeometricTransformation const &T_br, GeometricTransformation const &T_bo) {
    GeometricTransformation T_ro = homogeneneous_inverse(T_br)*T_bo;
    Vector3D axis = get_minus_z_axis(T_ro);
    axis.normalize();
    return axis;
}


RotationMatrix get_rotation(Vector3D x_orientation, Vector3D z) {
  z.normalize();
  x_orientation.normalize();
  Vector3D y = z.cross(x_orientation);
  y.normalize();
  Vector3D x = y.cross(z);

  RotationMatrix R;
  R << x, y, z;
  //INFO("R\n", R);
  return R;
}
RotationMatrix get_rotation_y(Vector3D y_orientation, Vector3D z) {
  z.normalize();
  y_orientation.normalize();
  Vector3D x = y_orientation.cross(z);
  x.normalize();
  Vector3D y = z.cross(x);

  RotationMatrix R;
  R << x, y, z;
  //INFO("R\n", R);
  return R;
}

GeometricTransformation fix_orientation(Vector3D const &on_object,
                                        Vector3D const &robot_pos,
                                        Vector3D const &x_orientation) {
  Vector3D z = on_object - robot_pos;
  RotationMatrix R = get_rotation(x_orientation, z);

  GeometricTransformation T_fin = translation(robot_pos) * rotation(R);
  return T_fin;
}

GeometricTransformation fix_orientation_y(Vector3D const &on_object,
                                          Vector3D const &robot_pos,
                                          Vector3D const &y_orientation) {
  Vector3D z = on_object - robot_pos;
  RotationMatrix R = get_rotation_y(y_orientation, z);

  GeometricTransformation T_fin = translation(robot_pos) * rotation(R);
  return T_fin;
}

Vector3D get_p3_for_config(double z, Config config) {

  //TBD - fix with BOX_POSITION
  double centerX = 0.77;
  double centerY = 0.1;
  double phi = 0.0;

  double diag = 47;
  double zero_angle = 0.56;

  float P1x = centerX - diag * sin(zero_angle - phi); //inside frame of the box closest to the robot
  float P1y = centerY - diag * cos(zero_angle - phi);
  float P2x = centerX + diag * sin(zero_angle + phi); //inside frame of the box at the far side of the robot
  float P2y = centerY - diag * cos(zero_angle + phi);

  float P3x = centerX - diag * sin(zero_angle + phi); //inside frame of the box closest to the robot
  float P3y = centerY + diag * cos(zero_angle + phi);
  float P4x = centerX + diag * sin(zero_angle - phi); //inside frame of the box at the far side of the robot
  float P4y = centerY + diag * cos(zero_angle - phi);
  

  //Test values (left half of the box - config = 2)
  switch (config)
  {
  case Config::c00 :
    return Vector3D(P1x, P1y, z);
  case Config::c01 :
    return Vector3D(P2x, P2y, z);
  case Config::c10 :
    return Vector3D(P3x, P3y, z);
  case Config::c11 :
    return Vector3D(P4x, P4y, z);
  //case Config::c01 :
  default:
    return Vector3D(P2x, P2y, z);
  //case Config::c10 : return Vector3D(P3x, P3y, z);
  //case Config::c11 : return Vector3D(P4x, P4y, z);
  }
}

Config get_configuration(Vector3D goal_pose, RobotPose current_pose){

  //Box parameters
  //TBD - fix with BOX_POSITION
  float centerX = 0.77;
  float centerY = 0.1;
  //phi - unimportant


  //Configuration hysteresis
  //TBD - implement hysteresis dependent on the current 
  //robot position and find the best configuration
  float width_hysteresis = 0.10;
  float lenght_hysteresis = 0.02;


 if (goal_pose(1) > centerY - width_hysteresis ) {
    if(goal_pose(0) > centerX){
      //CONFIGURATION FOR P1
      return Config::c00;
    } else {
      //CONFIGURATION FOR P2
      return Config::c01;
    }
  } else {
    if(goal_pose(0) > centerX){
      //CONFIGURATION FOR P3
      return Config::c10;
    } else {
      //CONFIGURATION FOR P4
      return Config::c11;
    }
  }
}


// maybe remove?
GeometricTransformation get_robot_pose_in_box(Vector3D const &p1, Vector3D const &p2, RobotPose const &current) {
  Vector3D p3 = get_p3_for_config(p1(2), get_configuration(p1, current));
  GeometricTransformation T = fix_orientation_y(p2, p1, p1 - p3);
  return T;
}

Vector3D get_object_translation_wrt_camera (GeometricTransformation const &T_br, GeometricTransformation const &T_bo) {
    GeometricTransformation T_ro = homogeneneous_inverse(T_br)*T_bo;
    Vector3D translation = T_ro.block<3,1>(0,3);
    return translation;
}

GeometricTransformation fix_shoulder_in_box_and_clamp(Vector3D p1, Vector3D const &p2, RobotPose const &current) {

  double centerX = 0.77;
  double centerY = 0.1;
  double phi = 0.0;

  //TBD - fix with BOX_POSITION
    double  box_x_min = centerX - 0.16, //0.60,
          box_x_max = centerX + 0.16, //0.93,
          box_y_min = centerY - 0.36, //-0.37, //Make it as big as possible
          box_y_max = centerY + 0.36; //0.37;
    p1 = Vector3D(
      clamp(p1(0), box_x_min, box_x_max),
      clamp(p1(1), box_y_min, box_y_max),
      p1(2)
      );
    return get_robot_pose_in_box(p1, p2, current);
}