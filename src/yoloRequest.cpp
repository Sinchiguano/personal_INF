/*
 * yoloRequest.cpp
 * Copyright (C) 2019 Cesar Sinchiguano <cesarsinchiguano@hotmail.es>
 *
 * Distributed under terms of the MIT license.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <iostream>
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

std_msgs::Bool yolo_request;
yolo_detector::BoundingBox data;
bool detectionDone = false;
yolo_detector::BoundingBox myBoundingBox;
bool controlState=false;
int counter1=0;
int counter2=0;


void yolo_detectorCallback(const yolo_detector::BoundingBox &msg){  data=msg;};
void yolo_detectionSignalCallback(const std_msgs::Bool::ConstPtr& msg){ detectionDone = msg->data;};
bool yolo_retrieveDetectionSignal(){return detectionDone; };
yolo_detector::BoundingBox yolo_retrieveData(){return data;};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker_");
  ros::NodeHandle n;


  ros::Publisher message_pub = n.advertise<std_msgs::String>("/message_topic", 1000);
  ros::Publisher signal_pub = n.advertise<std_msgs::Bool>("/signal_request_cpp", 1000);
  ros::Subscriber yolo_detector_sub = n.subscribe("/yolo_inference/bounding_box", 1000, yolo_detectorCallback);
  ros::Subscriber yolo_detectionSignal_sub = n.subscribe("/yolo_inference/detectionDone", 1000, yolo_detectionSignalCallback);

  image_transport::ImageTransport it(n);
  image_transport::Publisher image_pub = it.advertise("/camera_smartek/image_topic", 1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    counter1++;
    //Get the image from the camera and publish it
    Mat image = imread("/home/casch/ws_moveit/src/yolo_detector/script/tmp_img/000009.jpg"); // Read the file
    if( image.empty() )                      // Check for invalid input
    {
      cout <<  "Could not open or find the image" << std::endl ;
      return -1;
    }
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    yolo_request.data=true;
    controlState=false;
    counter2=0;
    while(!controlState)
    {
      counter2++;
      ros::Time time = ros::Time::now();
      cv_ptr->encoding = "bgr8";
      cv_ptr->header.stamp = time;
      cv_ptr->header.frame_id = "/camera_smartek/image_topic";
      cv_ptr->image = image;
      image_pub.publish(cv_ptr->toImageMsg());      //ROS_INFO("ImageMsg Send!");
      ros::spinOnce();

      controlState=yolo_retrieveDetectionSignal();
      myBoundingBox=yolo_retrieveData();



      if(counter2==10)
      {

        signal_pub.publish(yolo_request);
        ros::spinOnce();
      }
      ROS_INFO("Yolo_request is ON, now its value is  %d", yolo_request.data);
      std::cout<<"Counter2:"<<counter2<<std::endl;
    }


    //std::cout<<myBoundingBox<<std::endl;
    yolo_request.data=false;
    counter1=0;
    do {
      signal_pub.publish(yolo_request);
      ros::spinOnce();

      ROS_INFO("Yolo_request is OFF, now its value is  %d", yolo_request.data);
      std::cout<<"Counter1:"<<counter1<<std::endl;
      counter1++;
      sleep(0.3);

    } while( counter1 < 20 );

  }
  return 0;
}


//how to retrieve the data from the yoloDetector data
//   std::cout<<data.confidence<<std::endl;
//   std::cout<<data.topleft_x<<std::endl;
//   std::cout<<data.topleft_y<<std::endl;
//   std::cout<<data.bottomright_x<<std::endl;
//   std::cout<<data.bottomright_y<<std::endl;
//   std::cout<<data.Class<<std::endl;
