/*
 * yoloDetection.cpp
 * Copyright (C) 2019 Cesar Sinchiguano <cesarsinchiguano@hotmail.es>
 *
 * Distributed under terms of the MIT license.
 */
 //some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h> //ALWAYS need to include this

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

 // define a class, including a constructor, member variables and member functions
 class RosClass
 {
   public:
        //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
        RosClass(ros::NodeHandle* nodehandle);
        cv::Mat retrieveImage();

   private:
        // "private" data will only be available to member functions of this class;
        ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

        // some objects to support subscriber, service, and publisher
        ros::Subscriber pylon_subscriber_; //these will be set up within the class constructor, hiding these ugly details

        //place holder for the image from the ros topic

        cv::Mat tmpIMAGE;
        
        // member methods as well:
        // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
        void initializeSubscribers();

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
 }; // note: a class definition requires a semicolon at the end of the definition
