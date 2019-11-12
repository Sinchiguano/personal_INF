#include "ros/ros.h"
#include <cstdlib>

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


using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;
using namespace sensor_msgs;


#include "yolo_detector/AddTwoInts.h"
#include "yolo_detector/CheckForObjects.h"
#include "yolo_detector/BoundingBox.h"
#include "yolo_detector/YoloDetection.h"

// int main(int argc, char **argv)
// {

//   ros::init(argc, argv, "main_client");
//   ros::NodeHandle n;
//   ros::ServiceClient clientYoloTmp = n.serviceClient<yolo_detector::YoloDetection>("bounding_box_given_image");
//   yolo_detector::YoloDetection srvYoloTmp;


//   /////////////////////_YOLO DETECTOR_//////////////////////////////////////
//   Mat image = imread("/home/yumi/catkin_ws/src/yolo_detector/src/imageTmp/0018_0.jpg"); // Read the file
//   imwrite("/home/yumi/catkin_ws/src/yolo_detector/imageTest/tmpImage.png",image );
//   image = imread("/home/yumi/catkin_ws/src/yolo_detector/imageTest/tmpImage.png"); // Read the file

//   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
//   srvYoloTmp.request.ImageQuery = *msg;  //ROS_INFO("ImageMsg Send!");

//   if (clientYoloTmp.call(srvYoloTmp))
//   {
//     std::cout<<srvYoloTmp.response<<std::endl;
//   }
//   else
//   {
//     ROS_ERROR("Failed to call service detect_bounding_box");
//     return 1;
//   }


//   return 0;
// }


//std::cout<<"srvYolo.response.bounding_box: \n"<<srvYolo.response.bounding_box_tmp<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.confidence<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.topleft_x<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.topleft_y<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.bottomright_x<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.bottomright_y<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.Class<<std::endl;
