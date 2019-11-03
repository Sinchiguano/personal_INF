#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <sstream>
#include <iostream>

//image
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// using namespace cv;
// using namespace std;

//using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker_");
  ros::NodeHandle n;
  ros::Publisher message_pub = n.advertise<std_msgs::String>("/message_topic", 1000);
  ros::Publisher signal_pub = n.advertise<std_msgs::Bool>("/signal_topic", 1000);

  ros::Rate loop_rate(10);
  int count = 0;


  while (ros::ok())
  {
    std::stringstream ss;
    std_msgs::String msg;
    std_msgs::Bool yolo_request;

    //ss << "hello world " << count;
    ss << "hello world ";
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    message_pub.publish(msg);


    yolo_request.data=true;
    ROS_INFO("Yolo_request is ON, its value is %d", yolo_request.data);
    signal_pub.publish(yolo_request);
    sleep(2);
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
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
