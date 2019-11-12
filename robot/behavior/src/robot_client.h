/*
 * File name: robot_client.h
 * Date:      2016/11/10
 * Author:    Miroslav Kulich
 */

#ifndef __ROBOT_CLIENT_H
#define __ROBOT_CLIENT_H

#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "app_log.hpp"

#include "ur_planning.h"
#include "error_catch.hpp"

//#define ROBOT_HOSTNAME "192.168.0.2" // nuc
#define ROBOT_HOSTNAME "192.168.1.201" // pc
#define ROBOT_PORTNUM 6008
#define ROBOT_TRANSPORT_TCP 0
#define ROBOT_TRANSPORT_UDP 1

struct PoseMsg {
  double x, y, z, a, b, c;
  PoseMsg(double x, double y, double z, double a, double b, double c);
  PoseMsg(std::string const &str);
  std::string to_string() const;
};

namespace imr {

class RobotClientInterface {
public:
  virtual int set_pose(PoseMsg const &pose) = 0;
  virtual int set_joints_pose(std::vector<double> joints) = 0;
  virtual std::vector<double> get_joints() = 0;
  virtual void attach_item() = 0;
  virtual void item_vis(double x, double y,double z) = 0;
  virtual void detach_item(bool b) = 0;
  virtual void set_item_level(PoseMsg pose, bool b) = 0;
  virtual void set_vacuum(bool) = 0;
  virtual void set_light(int) = 0;
  virtual void set_force_mode(int) = 0;
  virtual PoseMsg get_pose() = 0;
  virtual std::vector<bool> vacuum_status() = 0;
};

class FakeRobotClient : public RobotClientInterface {
public:
  int set_pose(PoseMsg const &pose);
  int set_joints_pose(std::vector<double> joints);
  std::vector<double> get_joints();
  void attach_item();
  void item_vis(double x, double y,double z);
  void detach_item(bool b);
  void set_item_level(PoseMsg pose, bool b);
  void set_vacuum(bool b) {}
  void set_light(int i) {}
  void set_force_mode(int i) {}
  PoseMsg get_pose();
  std::vector<bool> vacuum_status();
};


class ROSRobotClient : public RobotClientInterface {
  UR_planning ur_plan;
public:
  ErrorPublisher timed_error_publisher_;

  int set_pose(PoseMsg const &pose);
  int set_joints_pose(std::vector<double> joints);
  std::vector<double> get_joints();
  void attach_item();
  void item_vis(double x, double y,double z);
  void detach_item(bool b);
  void set_item_level(PoseMsg pose, bool b);
  void set_vacuum(bool b);
  void set_light(int i);
  void set_force_mode(int i);
  PoseMsg get_pose();
  std::vector<bool> vacuum_status();
  ROSRobotClient();
};

class RobotClient : public RobotClientInterface {
public:
  /// Make a client and connect it as indicated.
  RobotClient(const std::string aHostname = ROBOT_HOSTNAME,
              int aPort = ROBOT_PORTNUM, int aTransport = ROBOT_TRANSPORT_TCP);
  /// destructor
  ~RobotClient();

  //client stuff
  void sendMessage(std::string msg);
  void stop();
  std::string receive();
  //  bool receive(CStatus &status);

  // control stuff. TODO move from here to RobotController
  int set_pose(PoseMsg const &pose);
  int set_joints_pose(std::vector<double> joints);
  std::vector<double> get_joints();
  PoseMsg get_pose();
  std::vector<bool> vacuum_status();
  void attach_item();
  void item_vis(double x, double y,double z);
  void detach_item(bool b);
  void set_item_level(PoseMsg pose, bool b);
  void set_vacuum(bool);
  void set_light(int);
  void set_beacon_string(std::string const &str);
  void set_force_mode(int i) {}
private:
  const std::string ipAddress;
  const int serverPort;
  int socketFd;
  //   std::thread recThread;
  //   std::mutex quitMutex;
  bool quit;

  bool connect();
};

class HybridRobotClient : public RobotClientInterface {
  std::shared_ptr<ROSRobotClient> ros;
  std::shared_ptr<RobotClientInterface> pn;
public:
  int set_pose(PoseMsg const &pose);
  int set_joints_pose(std::vector<double> joints);
  std::vector<double> get_joints();
  void attach_item();
  void item_vis(double x, double y,double z);
  void detach_item(bool b);
  void set_item_level(PoseMsg pose, bool b);
  void set_vacuum(bool b);
  void set_light(int i);
  void set_force_mode(int i);
  PoseMsg get_pose();
  std::vector<bool> vacuum_status();
  HybridRobotClient(std::shared_ptr<RobotClientInterface> pn_client);
};

} // namespace imr
#endif // ROBOT_CLIENT_H
