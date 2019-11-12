#include <ros/ros.h>
#include <exception>

// create hog data
void create_hog_data();
void show_bounding_boxes();

enum class HOGCommand { create_pose_data, create_occlusion_data, error };

HOGCommand parse_hog_command()
{
  ros::NodeHandle nh("~");
  std::string task;
  nh.param<std::string>("task", task, "create_pose_data");
  // clang-format off
  HOGCommand cmd =
      task == "create_pose_data" ? HOGCommand::create_pose_data
      : task == "create_occlusion_data" ? HOGCommand::create_occlusion_data
      : HOGCommand::error;
  // clang-format on
  return cmd;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hog");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  auto cmd = parse_hog_command();
  switch (cmd)
  {
  case HOGCommand::create_pose_data:
    create_hog_data();
    break;
  case HOGCommand::create_occlusion_data:
    show_bounding_boxes();
    break;
  case HOGCommand::error:
    std::cout << "HOGCommand unknown" << std::endl;
  default:
    break;
  }

  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  std::terminate();
}
