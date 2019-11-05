#include "ros/ros.h"
#include "yolo_detector/AddTwoInts.h"
#include "yolo_detector/CheckForObjects.h"
#include <cstdlib>
#include "yolo_detector/BoundingBox.h"


//for openCV, i have all i need je je je
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

//for publishing image through the ros topic
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "main_client");
  ros::NodeHandle n;
  ros::ServiceClient clientYolo = n.serviceClient<yolo_detector::CheckForObjects>("detect_bounding_box");
  yolo_detector::CheckForObjects srvYolo;
  srvYolo.request.data = true;


  /////////////////////_YOLO DETECTOR_//////////////////////////////////////
  Mat image = imread("/home/casch/ws_moveit/src/yolo_detector/src/imageTmp/0018_0.jpg"); // Read the file
  imwrite( "/home/casch/ws_moveit/src/yolo_detector/imageTest/tmpImage.png",image );
  image = imread("/home/casch/ws_moveit/src/yolo_detector/imageTest/tmpImage.png"); // Read the file

  if (clientYolo.call(srvYolo))
  {
    std::cout<<srvYolo.response<<std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service detect_bounding_box");
    return 1;
  }
  ////////////////////////////////////////////////////////////



  return 0;
}


//std::cout<<"srvYolo.response.bounding_box: \n"<<srvYolo.response.bounding_box_tmp<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.confidence<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.topleft_x<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.topleft_y<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.bottomright_x<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.bottomright_y<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.Class<<std::endl;
