#ifndef BOX_INFO_HPP
#define BOX_INFO_HPP

#include <robot_measurement.hpp>

struct BoxPose {
    // center
    double x, y;
    double phi;
    BoxPose(double _x, double _y, double _phi) : x{_x}, y{_y}, phi{_phi} {}
    BoxPose(PoseImgMeasurements const &measurements) {}
};

#endif //!BOX_INFO_HPP
