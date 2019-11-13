/*
 * yoloDetection.cpp
 * Copyright (C) 2019 Cesar Sinchiguano <cesarsinchiguano@hotmail.es>
 *
 * Distributed under terms of the MIT license.
 */
 //some generically useful stuff to include...

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "pylon_class.h"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
  // ROS set-ups:
  ros::init(argc, argv, "Ros_pylonNode"); //node name
  ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

  ROS_INFO("main: Instantiating an object of type RosClass");
  RosClass pylonObject(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
  cv::Mat image;
  ros::Rate loop_rate(10);
  ros::spinOnce();

  int counter=0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    counter++;
    image = pylonObject.retrieveImage();
    std::cout<<"counter: "<<counter<<endl;

    // Check for invalid input
    if(! image.data )
    {
       cout <<  "Could not open or find the image" << std::endl ;
       continue;
    }
    //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.
    waitKey(30);


  }

  //cv::destroyWindow("Display window");
  return 0;
}
