#include "ros/ros.h"
#include "yolo_detector/AddTwoInts.h"
#include "yolo_detector/CheckForObjects.h"
#include <cstdlib>
#include "yolo_detector/BoundingBox.h"

int main(int argc, char **argv)
{
  int tmp1=2;
  int tmp2=4;
  ros::init(argc, argv, "main_client");

  ros::NodeHandle n;
  ros::ServiceClient clientAdd = n.serviceClient<yolo_detector::AddTwoInts>("add_two_ints");
  ros::ServiceClient clientYolo = n.serviceClient<yolo_detector::CheckForObjects>("detect_bounding_box");
  yolo_detector::AddTwoInts srvAdd;
  yolo_detector::CheckForObjects srvYolo;

  // srvAdd.request.a = tmp1;
  // srvAdd.request.b = tmp2;
  srvYolo.request.trick = 2;

  if (clientYolo.call(srvYolo))
  {
    std::cout<<srvYolo.response.bounding_box_tmp.confidence<<std::endl;
    std::cout<<srvYolo.response.bounding_box_tmp.topleft_x<<std::endl;
    std::cout<<srvYolo.response.bounding_box_tmp.topleft_y<<std::endl;
    std::cout<<srvYolo.response.bounding_box_tmp.bottomright_x<<std::endl;
    std::cout<<srvYolo.response.bounding_box_tmp.bottomright_y<<std::endl;
    std::cout<<srvYolo.response.bounding_box_tmp.Class<<std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service detect_bounding_box");
    return 1;
  }
  return 0;
}



// if (clientAdd.call(srvAdd))
// {
//   ROS_INFO("Sum: %ld", (long int)srvAdd.response.sum);
// }
// else
// {
//   ROS_ERROR("Failed to call service add_two_ints");
//   return 1;
// }
