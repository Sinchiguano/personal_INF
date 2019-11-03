#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <iostream>
#include "yolo_detector/BoundingBox.h"
#include <opencv2/opencv.hpp>

//image
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// using namespace cv;
//using namespace std;
using namespace std;


yolo_detector::BoundingBox data;
bool detectionDone = false;

void yolo_detectorCallback(const yolo_detector::BoundingBox &msg)
{
  // ROS_INFO("I heard from /yolo_inference/bounding_box topic:");
  // std::cout<<msg<<std::endl;
  // std::cout<<"Bouding Boxes (confidence ):" << msg.confidence <<std::endl;
  // cout<<"Bouding Boxes (topleft_x ):" << msg.topleft_x <<endl;
  // cout<<"Bouding Boxes (topleft_y ):" << msg.topleft_y <<endl;
  // cout<<"Bouding Boxes (bottomright_x):" << msg.bottomright_x <<endl;
  // cout<<"Bouding Boxes (bottomright_y):" << msg.bottomright_y<<endl;
  // cout<<"Bouding Boxes (Class):" << msg.Class<<endl;
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
    //cout<<"////////////////////////////"<<endl;

    /*waiting until the yolo detector is done*/
    // Mat image;
    // image = imread("/home/casch/ws_moveit/src/yolo_inference/script/result/*.pgn", CV_LOAD_IMAGE_COLOR);   // Read the file
    // if(! image.data )                              // Check for invalid input
    // {
    //   cout <<  "Could not open or find the image" << std::endl ;
    //   return -1;
    // }
    // namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    // imshow( "Display window", image );                   // Show our image inside it.
    // waitKey(0);
    /**/

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
