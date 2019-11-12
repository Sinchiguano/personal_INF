
#ifndef __ROBOT_CLIENT_ISOLATED_H
#define __ROBOT_CLIENT_ISOLATED_H

#include <functional>
#include <string>

struct RobotClientIsolated
{
  using set_beacon_cb = std::function<void(std::string const &)>;
  set_beacon_cb set_beacon_;
  RobotClientIsolated(set_beacon_cb const &set_beacon) : set_beacon_(set_beacon) {}
};
RobotClientIsolated makeRobotClientIsolated();

#endif // ROBOT_CLIENT_ISOLATED_H
