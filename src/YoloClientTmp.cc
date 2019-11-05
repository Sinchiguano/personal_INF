#include "ros/ros.h"
#include "yolo_detector/AddTwoInts.h"
#include "yolo_detector/CheckForObjects.h"
#include <cstdlib>
#include "yolo_detector/BoundingBox.h"
#include "yolo_detector/YoloDetection.h"

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
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


#include <cv_bridge/cv_bridge.h>//#include "cv_bridge/CvBridge.h" the later one does work at all.


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;
using namespace sensor_msgs;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "main_client");
  ros::NodeHandle n;
  ros::ServiceClient clientYolo = n.serviceClient<yolo_detector::CheckForObjects>("detect_bounding_box");
  yolo_detector::CheckForObjects srvYolo;
  ros::ServiceClient clientYoloTmp = n.serviceClient<yolo_detector::YoloDetection>("bounding_box_given_image");
  yolo_detector::YoloDetection srvYoloTmp;


  /////////////////////_YOLO DETECTOR_//////////////////////////////////////
  Mat image = imread("/home/casch/ws_moveit/src/yolo_detector/src/imageTmp/0017_0.jpg"); // Read the file
  imwrite( "/home/casch/ws_moveit/src/yolo_detector/imageTest/tmpImage.png",image );
  image = imread("/home/casch/ws_moveit/src/yolo_detector/imageTest/tmpImage.png"); // Read the file

  srvYolo.request.data = true;


  // cv_bridge::CvImage cv_msg;
  // ros::Time time = ros::Time::now();
  // cv_msg.header.stamp = time;
  // cv_msg.header.frame_id = "/camera_smartek/image_topic";
  // cv_msg.encoding = "bgr8";
  // cv_msg.image = image; // Your cv::Mat
  // cv_msg.toImageMsg();
  // //saliency_img_pub.publish(cv_msg.toImageMsg());//publishing no problem
  // //srvYoloTmp.request.ImageQuery = cv_msg;  //ROS_INFO("ImageMsg Send!");


  //cv::Mat image_ = cv::imread("/home/casch/ws_moveit/src/yolo_detector/src/imageTmp/lion.jpg", CV_LOAD_IMAGE_COLOR);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  srvYoloTmp.request.ImageQuery = *msg;  //ROS_INFO("ImageMsg Send!");

  // if (clientYolo.call(srvYolo))
  // {
  //   std::cout<<srvYolo.response<<std::endl;
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service detect_bounding_box");
  //   return 1;
  // }
  ////////////////////////////////////////////////////////////
  if (clientYoloTmp.call(srvYoloTmp))
  {
    std::cout<<srvYoloTmp.response<<std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service detect_bounding_box");
    return 1;
  }


  return 0;
}


//std::cout<<"srvYolo.response.bounding_box: \n"<<srvYolo.response.bounding_box_tmp<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.confidence<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.topleft_x<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.topleft_y<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.bottomright_x<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.bottomright_y<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.Class<<std::endl;
