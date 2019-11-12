/*
 * File name: robot_client.cc
 * Date:      2016/11/10
 * Author:    Miroslav Kulich
 */

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <netdb.h>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>

#include <errno.h>

#include "app_log.hpp"
#include "convert_3d.hpp"
#include "robot_client.h"
#include "utilities.hpp"

#include "log_publisher.hpp"

#include "ur_planning.h"
#include <tf/transform_listener.h>

using namespace imr;

double x, y, z, a, b, c;

PoseMsg::PoseMsg(double x, double y, double z, double a, double b, double c)
    : x(x), y(y), z(z), a(a), b(b), c(c) {}

PoseMsg::PoseMsg(std::string const &str)
{
  std::istringstream ss(str);
  // std::stringstream ph;
  std::string ph;
  ss >> ph;
  ss >> ph;
  ss >> x;
  ss >> y;
  ss >> z;
  ss >> a;
  ss >> b;
  ss >> c;
}

std::string PoseMsg::to_string() const
{
  std::stringstream ss;
  // clang-format off
      ss  << x << " "
          << y << " "
          << z << " "
          << a << " "
          << b << " "
          << c;
  // clang-format on
  return ss.str();
}

int FakeRobotClient::set_pose(PoseMsg const &pose)
{
  INFO("FakeRobotClient: set_pose ", pose.to_string());
  return -5;
}

int FakeRobotClient::set_joints_pose(std::vector<double> joints)
{
  INFO("FakeRobotClient: set_joints_pose ");
  return -5;
}

std::vector<double> FakeRobotClient::get_joints()
{
  INFO("FakeRobotClient: get_joints ");
  std::vector<double> joints;
  return joints;
}

PoseMsg FakeRobotClient::get_pose()
{
  INFO("FakeRobotClient: get_pose ");
  return PoseMsg(0, 0, 0, 0, 0, 0);
}

void FakeRobotClient::attach_item() { INFO("FakeRobotClient: attach_item "); }

void FakeRobotClient::item_vis(double x, double y, double z)
{
  INFO("FakeRobotClient: item_vis ");
}

void FakeRobotClient::detach_item(bool b)
{
  INFO("FakeRobotClient: detach_item ");
}

void FakeRobotClient::set_item_level(PoseMsg pose, bool b)
{
  INFO("FakeRobotClient: set_item_level ");
}

std::vector<bool> FakeRobotClient::vacuum_status()
{
  INFO("FakeRobotClient: vacuum_status ");
  std::vector<bool> dummy;
  return dummy;
}

bool valid_joint_positions(std::vector<double> joints)
{
  for (int i = 0; i < joints.size(); i++)
  {
    if (fabs(joints.at(i)) > M_PI)
    {
      return false;
    }
  }
  return true;
}

int ROSRobotClient::set_pose(PoseMsg const &pose)
{
  //Test joint range before movement
  auto current_joints = ur_plan.get_joints();
  if (!valid_joint_positions(current_joints))
  {
    JointErr jErr(&timed_error_publisher_);
    INFO("joints : ", current_joints[0], ", ", current_joints[1],
         ", ", current_joints[2], ", ", current_joints[3], ", ",
         current_joints[4], ", ", current_joints[5]);
  }

  TimeBomb bomb(30000, &timed_error_publisher_, 1);
  geometry_msgs::Pose t_pose;
  ScaledAxisAngle saa(pose.a, pose.b, pose.c);
  Quaternion q = toQuaternion(saa);
  t_pose.position.x = pose.x;
  t_pose.position.y = pose.y;
  t_pose.position.z = pose.z;
  t_pose.orientation.w = q(0);
  t_pose.orientation.x = q(1);
  t_pose.orientation.y = q(2);
  t_pose.orientation.z = q(3);

  LoggerPub log("Set_pose " +   pose.to_string());
  int out = ur_plan.moveTo(t_pose);
  LoggerPub log2("Set_pose_finished");
  return out;
}

int ROSRobotClient::set_joints_pose(std::vector<double> joints)
{
  //Test joint range before movement
  auto current_joints = ur_plan.get_joints();
  if (!valid_joint_positions(current_joints))
  {
    JointErr jErr(&timed_error_publisher_);
    INFO("joints : ", current_joints[0], ", ", current_joints[1],
         ", ", current_joints[2], ", ", current_joints[3], ", ",
         current_joints[4], ", ", current_joints[5]);
  }

  TimeBomb bomb(30000, &timed_error_publisher_, 1);

  std::stringstream ss;
  ss << "Set_joints_pose " << joints.at(0) << ", " << joints.at(1) << ", " 
    << joints.at(2) << ", " << joints.at(3) << ", " << joints.at(4) << ", " 
    << joints.at(5);
  LoggerPub log(ss.str());

  auto tmp = ur_plan.moveTo_joints(joints);
  LoggerPub log2("Set_joints_pose_finished");
  return tmp;
}

std::vector<double> ROSRobotClient::get_joints()
{

  return ur_plan.get_joints();
}

void ROSRobotClient::attach_item() { ur_plan.attachItem(); }

void ROSRobotClient::item_vis(double x, double y, double z)
{
  ur_plan.item_vis(x, y, z);
}

void ROSRobotClient::detach_item(bool b)
{

  //TODO - remove this ultimate hack
  //ur_plan.initPlanningScene(); //(removed)

  ur_plan.detachItem(b);
}

void ROSRobotClient::set_item_level(PoseMsg pose, bool b)
{
  ur_plan.set_item_level(pose.x, pose.y, pose.z, b);
}

void ROSRobotClient::set_vacuum(bool b) { 
  if(b){
    LoggerPub log("Set_vacuum ON");
  }
  else{
    LoggerPub log("Set_vacuum OFF");
  }

  ur_plan.suction(b); 
  }

void ROSRobotClient::set_light(int i) { 

    LoggerPub log("Set_light " + std::to_string(i));

  ur_plan.light(uint8_t(i)); 
  }

void ROSRobotClient::set_force_mode(int i) { 
  ur_plan.setForceMode(i);
  }

RotationMatrix toRotationMatrix(tf::Matrix3x3 const &rosmat)
{
  RotationMatrix mat;
}

PoseMsg ROSRobotClient::get_pose()
{

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    listener.waitForTransform("/base_link", "/ee_link", ros::Time(0),
                              ros::Duration(3.0));
    listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  double x = transform.getOrigin().x();
  double y = transform.getOrigin().y();
  double z = transform.getOrigin().z();

  tf::Matrix3x3 m(transform.getRotation());
  double roll, pitch, yaw;
  m.getEulerYPR(yaw, pitch, roll, 1);

  double Qx = transform.getRotation().x();
  double Qy = transform.getRotation().y();
  double Qz = transform.getRotation().z();
  double Qw = transform.getRotation().w();
  Quaternion q(Qw, Qx, Qy, Qz);
  ScaledAxisAngle saa = toScaledAxisAngle(q);
  RobotPose rp(x, y, z, saa(0), saa(1), saa(2));
  /*
    ROS_INFO("\nX    : %.3f\n"
            "Y    : %.3f\n"
            "Z    : %.3f\n"
            "Rx    : %.3f\n"
            "Ry  : %.3f\n"
            "Rz    : %.3f\n",
            x, y, z, saa(0), saa(1), saa(2) );
  */

  return toPoseMsg(rp);
}

std::vector<bool> ROSRobotClient::vacuum_status()
{
  return ur_plan.vacuum_status();
}

ROSRobotClient::ROSRobotClient() : timed_error_publisher_{}
{
  std::cout << "INIT Planning scene" << std::endl;
  ur_plan.initPlanningScene();
  std::cout << "Planning scene INITIALIZED" << std::endl;
  // ur_plan.attachItem();
}

//
RobotClient::RobotClient(const std::string aHostname, int aPort, int aTransport)
    : ipAddress(aHostname), serverPort(aPort), socketFd(-1), quit(false)
{
  connect();
  //   recThread =  std::thread(&RobotClient::receive,this);
}

RobotClient::~RobotClient()
{
  stop();
  //   DEBUG("Waiting for recThread.");
  //   recThread.join();
  //   DEBUG("recThread finished.");
  ::shutdown(socketFd, SHUT_RDWR);
}

bool RobotClient::connect()
{
  bool ipv6 = false;
  bool connected = false;
  do
  {
    socketFd = ::socket(ipv6 ? AF_INET6 : AF_INET, SOCK_STREAM, 0);
    if (socketFd < 0)
    {
      // ERROR("Failed to create socket: " << strerror(errno));
      ERROR("Failed to create socket: ", strerror(errno));
      break;
    }

    const struct sockaddr *addr;

    struct hostent *host_info;
    host_info = gethostbyname(ipAddress.c_str());
    if (host_info == NULL)
    {
      ERROR("Failed to resolve host name '", ipAddress.c_str(),
            "' : ", strerror(errno));
      break;
    }

    int addr_size;
    if (!ipv6)
    {
      struct sockaddr_in *addr4 = new struct sockaddr_in;
      addr4->sin_family = PF_INET;
      addr4->sin_port = htons(serverPort);
      memcpy(&addr4->sin_addr.s_addr, host_info->h_addr, host_info->h_length);
      addr = (const struct sockaddr *)addr4;
      addr_size = sizeof(struct sockaddr_in);
    }
    else
    {
      struct sockaddr_in6 *addr6 = new struct sockaddr_in6;
      addr6->sin6_family = PF_INET6;
      addr6->sin6_port = htons(serverPort);
      memcpy((char *)&(addr6->sin6_addr.s6_addr), host_info->h_addr_list[0],
             host_info->h_length);
      addr = (const struct sockaddr *)addr6;
      addr_size = sizeof(struct sockaddr_in);
    }
    INFO("connecting to robot ", ipAddress, ":", serverPort, " ...");
    auto connect_code = ::connect(socketFd, addr, addr_size);
    if (connect_code < 0)
    {
      ERROR("Connect error: ", strerror(errno));
    }
    INFO("connect_code ", connect_code);
    if (connect_code == 0)
    {
      INFO("connected to robot ", ipAddress, ":", serverPort);
      connected = true;
    }
    else
    {
      ERROR("Failed to connect to robot at ", ipAddress, ":", serverPort);
    }
    delete addr;
  } while (false);

  // set timeout for the socket receiver
  struct timeval timeout;
  timeout.tv_sec = 1; // SOCKET_READ_TIMEOUT_SEC;
  timeout.tv_usec = 0;
  setsockopt(socketFd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  return connected;
}

void RobotClient::stop()
{
  //   std::unique_lock<std::mutex> lk(quitMutex);
  quit = true;
}

std::string RobotClient::receive()
{
  bool q = false;
  LoggerPub log("CLIENT_RECEIVING_MSG ");
  std::stringstream ss;
  while (!q)
  {
    char b;
    if (::recv(socketFd, &b, 1, 0) == 1)
    {
      ss << b;
      if (int(b) == 0)
      {
        q = true;
      }
    }
    //     std::unique_lock<std::mutex> lk(quitMutex);
    q |= quit;
  }
  INFO("recv end ", ss.str());
  LoggerPub log2("CLIENT_RECEIVED_MSG");// + ss.str());
  return ss.str();
}

void RobotClient::sendMessage(std::string msg)
{
  LoggerPub log("CLIENT_SENDING_MSG");// + msg);
  INFO("robotclient send message ", msg);
  if (::send(socketFd, msg.c_str(), msg.length() + 1, 0) < 0)
  {
    ERROR("send error: ", strerror(errno));
  }
  LoggerPub log2("CLIENT_SENT_MSG");// + msg);
}

int RobotClient::set_pose(PoseMsg const &pose)
{
  //auto msg = pose.to_string();
  //sendMessage(msg);
  //receive();
  return -5;
}

int RobotClient::set_joints_pose(std::vector<double> joints) { return -5; }

std::vector<double> RobotClient::get_joints()
{
  std::vector<double> joints;
  return joints;
}

PoseMsg RobotClient::get_pose()
{
  auto msg = "get_pose";
  sendMessage(msg);
  auto resp = receive();
  INFO("get_pose responce: ", resp);
  return PoseMsg(resp);
}

std::vector<bool> RobotClient::vacuum_status()
{
  /*
  auto msg = "vacuum_status";
  sendMessage(msg);
  auto resp = receive();
  // INFO("vacuum_status response: ", resp);

  std::istringstream ss(resp);
  // std::stringstream ph;
  std::vector<URVacuumStatus> status;
  URVacuumStatus v0, v1, v2, v3, v4, v5;
  std::string ph;
  ss >> ph;
  ss >> v0.raw;
  status.push_back(v0);
  ss >> v1.raw;
  status.push_back(v1);
  ss >> v2.raw;
  status.push_back(v2);
  ss >> v3.raw;
  status.push_back(v3);
  ss >> v4.raw;
  status.push_back(v4);
  ss >> v5.raw;
  status.push_back(v5);
  */
   std::vector<bool> status;
  return status;
}

void RobotClient::attach_item() {}

void RobotClient::item_vis(double x, double y, double z) {}

void RobotClient::detach_item(bool b) {}

void RobotClient::set_item_level(PoseMsg pose, bool b) {}

void RobotClient::set_vacuum(bool b)
{
  std::stringstream ss;
  ss << "set_vacuum " << b;
  sendMessage(ss.str());
  receive();
}

void RobotClient::set_light(int l)
{
  delay(50);
  std::stringstream ss;
  ss << "set_light " << l;
  sendMessage(ss.str());
  receive();
}
void RobotClient::set_beacon_string(std::string const &str)
{
  delay(50);
  sendMessage(str);
  receive();
}

int HybridRobotClient::set_pose(PoseMsg const &pose)
{

  return ros->set_pose(pose);
}

int HybridRobotClient::set_joints_pose(std::vector<double> joints)
{
  return ros->set_joints_pose(joints);
}

std::vector<double> HybridRobotClient::get_joints()
{
  return ros->get_joints();
}

void HybridRobotClient::attach_item()
{
  delay(200);
  ros->attach_item();
  delay(200);
}

void HybridRobotClient::item_vis(double x, double y, double z)
{
  ros->item_vis(x, y, z);
}

void HybridRobotClient::detach_item(bool b)
{
  delay(50);
  ros->detach_item(b);
  delay(50);
}

void HybridRobotClient::set_item_level(PoseMsg pose, bool b)
{
  delay(50);
  ros->set_item_level(pose, b);
  delay(50);
}

void HybridRobotClient::set_vacuum(bool b) { ros->set_vacuum(b); }

void HybridRobotClient::set_light(int i)
{
  ros->set_light(i);
  // pn->set_light(i);
}

PoseMsg HybridRobotClient::get_pose() { return ros->get_pose(); }

std::vector<bool> HybridRobotClient::vacuum_status()
{
  return ros->vacuum_status();
}

void HybridRobotClient::set_force_mode(int i) { 
  ros->set_force_mode(i);
  }

HybridRobotClient::HybridRobotClient(
    std::shared_ptr<RobotClientInterface> pn_client)
    : ros{std::make_shared<ROSRobotClient>()}, pn{pn_client} 
{
std::cout << "Hybrid initialize" << std::endl;
}



#include "robot_client_isolated.h"
RobotClientIsolated makeRobotClientIsolated()
{
  auto robot_client = std::make_shared<RobotClient>();
  auto cb = [robot_client](std::string const &str) {
    robot_client->set_beacon_string(str);
  };
  return RobotClientIsolated(cb);
}