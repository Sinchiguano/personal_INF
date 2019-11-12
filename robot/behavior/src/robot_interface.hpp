#ifndef ROBOT_INTERFACE
#define ROBOT_INTERFACE

#include <memory>

enum class RobotCommand {debug, make_data, make_occlusion_data, grasping, home, init};
RobotCommand parseRobotCommand(std::string const &cmd_str);

struct Img {};
struct InitialImgs {
  Img a;
  Img b;
};

struct start_approaching_args {
  std::shared_ptr<InitialImgs> ii;
};

struct start_grasping_args {};
struct recover_approaching_args {};
struct recover_grasping_args {};
struct start_putting_args {};

// TODO remove interface
struct RobotInterface {
  virtual bool a_go_home() = 0;
  virtual bool a_init_localize() = 0;
  virtual bool a_do_manual(RobotCommand cmd) = 0;
  virtual bool a_making_initial_imgs() = 0;
  virtual bool a_start_approaching(start_approaching_args args) = 0;
  virtual bool a_recover_approaching(recover_approaching_args args) = 0;
  virtual bool a_recover_grasping(recover_grasping_args args) = 0;
  virtual bool a_start_grasping(start_grasping_args args) = 0;
  virtual bool a_start_putting(start_putting_args args) = 0;
};

#endif
