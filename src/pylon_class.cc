/*
 * yoloDetection.cpp
 * Copyright (C) 2019 Cesar Sinchiguano <cesarsinchiguano@hotmail.es>
 *
 * Distributed under terms of the MIT license.
 */
 // this header incorporates all the necessary #include files and defines the class "RosClass"
 #include "pylon_class.h"
 #include <ros/ros.h>
 #include <image_transport/image_transport.h>
 #include <opencv2/highgui/highgui.hpp>
 #include <cv_bridge/cv_bridge.h>
 //CONSTRUCTOR:  this will get called whenever an instance of this class is created
 RosClass::RosClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
 { // constructor
     ROS_INFO("in class constructor of ExampleRosClass");
     initializeSubscribers();

 }
 //member helper function to set up subscribers;
 void RosClass::initializeSubscribers()
 {
     ROS_INFO("Initializing Subscribers");
     pylon_subscriber_ = nh_.subscribe("/pylon_camera_node/image_raw", 1,&RosClass::imageCallback,this);
 }

 void RosClass::imageCallback(const sensor_msgs::ImageConstPtr& msg)
 {
   try
   {
     tmpIMAGE= cv_bridge::toCvCopy(msg, "bgr8")->image;
     //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
     //cv::imshow("view", tmpIMAGE);
     //cv::waitKey(30);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
 }
cv::Mat RosClass::retrieveImage()
{
  //std::cout<<"just a tine message"<<std::endl;
  return tmpIMAGE;
}

// int main(int argc, char** argv)
// {
//      // ROS set-ups:
//      ros::init(argc, argv, "Ros_pylonNode"); //node name
//      ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
//
//      ROS_INFO("main: Instantiating an object of type RosClass");
//      RosClass pylonObject(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
//
//
//      ros::spin();
//      return 0;
//  }
