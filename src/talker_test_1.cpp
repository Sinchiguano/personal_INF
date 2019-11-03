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


using namespace cv;
using namespace std;


yolo_detector::BoundingBox data;
bool detectionDone = false;

void yolo_detectorCallback(const yolo_detector::BoundingBox &msg)
{
  data=msg;
}

yolo_detector::BoundingBox yolo_retrieveData(){return data;};
void yolo_detectionSignalCallback(const std_msgs::Bool::ConstPtr& msg){ detectionDone = msg->data;}
bool yolo_retrieveDetectionSignal(){return detectionDone; };

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker_");
  ros::NodeHandle n;
  ros::Publisher message_pub = n.advertise<std_msgs::String>("/message_topic", 1000);
  ros::Publisher signal_pub = n.advertise<std_msgs::Bool>("/signal_request_cpp", 1000);
  ros::Subscriber yolo_detector_sub = n.subscribe("/yolo_inference/bounding_box", 1000, yolo_detectorCallback);
  ros::Subscriber yolo_detectionSignal_sub = n.subscribe("/yolo_inference/detectionDone", 1000, yolo_detectionSignalCallback);


  ros::Rate loop_rate(10);
  int count = 0;
  bool controlState=false;

  while (ros::ok())
  {
    std::stringstream ss;
    std_msgs::String msg;
    std_msgs::Bool yolo_request;

    //ss << "hello world " << count;
    // ss << "hello world ";
    // msg.data = ss.str();
    // ROS_INFO("%s", msg.data.c_str());
    // message_pub.publish(msg);

    //get image from the camera
    Mat image = imread("/home/casch/ws_moveit/src/yolo_detector/src/testImage.png"); // Read the file
    if( image.empty() )                      // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    //save image for later use of the yolo detector
    Mat gray_image;
    cvtColor( image, gray_image, CV_BGR2GRAY );
    imwrite( "/home/casch/ws_moveit/src/yolo_detector/script/tmp_img/testImage_copy.png", gray_image );

    // namedWindow( "testImage.png", CV_WINDOW_AUTOSIZE );
    // namedWindow( "Gray image", CV_WINDOW_AUTOSIZE );
    // imshow( "testImage.png", image );
    // imshow( "Gray image", gray_image );

    //waitKey(0);





    yolo_request.data=true;
    ROS_INFO("Yolo_request is ON, its value is %d", yolo_request.data);
    signal_pub.publish(yolo_request);
    ros::spinOnce();
    yolo_detector::BoundingBox someCrap;
    while(!controlState)
    {
      cout<<"waiting for the answer of Yolo detector!"<<endl;
      controlState=yolo_retrieveDetectionSignal();
      // if (controlState==true)
      // {
      //   //yolo_request.data=false;
      //   //signal_pub.publish(yolo_request);
      //   ros::spinOnce();
      // }
      signal_pub.publish(yolo_request);
      ros::spinOnce();
    }
    //cout<<"////////////////////////////"<<endl;
    std::cout<<yolo_retrieveData()<<std::endl;
    sleep(4);


    yolo_request.data=false;
    ROS_INFO("Yolo_request is OFF, now its value is  %d", yolo_request.data);
    signal_pub.publish(yolo_request);
    std::cout<<someCrap<<std::endl;
    sleep(4);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
