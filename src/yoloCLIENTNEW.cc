#include "ros/ros.h"
#include "yolo_detector/AddTwoInts.h"
#include "yolo_detector/CheckForObjects.h"
#include "yolo_detector/BoundingBox.h"
#include "yolo_detector/YoloDetection.h"
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


// C++ program to demonstrate the use of std::min
#include <algorithm>
#include "geometry_msgs/Point.h"
//#include <bits/stdc++.h>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;
using namespace sensor_msgs;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "main_client");
  ros::NodeHandle n;
  ros::ServiceClient clientYoloTmp = n.serviceClient<yolo_detector::YoloDetection>("bounding_box_given_image");
  yolo_detector::YoloDetection srvYoloTmp;


  /////////////////////_YOLO DETECTOR_//////////////////////////////////////
  Mat image = imread("/home/casch/ws_moveit/src/yolo_detector/src/imageTmp/0024_0.jpg"); // Read the file
  imwrite( "/home/casch/ws_moveit/src/yolo_detector/imageTest/tmpImage.png",image );
  image = imread("/home/casch/ws_moveit/src/yolo_detector/imageTest/tmpImage.png"); // Read the file

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  srvYoloTmp.request.ImageQuery = *msg;  //ROS_INFO("ImageMsg Send!");

  cv::Rect yoloBoxCv;
  yolo_detector::BoundingBox yoloData;
  if (clientYoloTmp.call(srvYoloTmp))
  {
    //std::cout<<srvYoloTmp.response<<std::endl;
    std::cout<<"-------------------------"<<std::endl;
    std::cout<<srvYoloTmp.response.BoundingBoxAnswer<<std::endl;
    yoloData.topleft_x=srvYoloTmp.response.BoundingBoxAnswer.topleft_x;
    yoloData.topleft_y=srvYoloTmp.response.BoundingBoxAnswer.topleft_y;
    yoloData.bottomright_x=srvYoloTmp.response.BoundingBoxAnswer.bottomright_x;
    yoloData.bottomright_y=srvYoloTmp.response.BoundingBoxAnswer.bottomright_y;
    yoloData.id=srvYoloTmp.response.BoundingBoxAnswer.id;
  }
  else
  {
    ROS_ERROR("Failed to call service detect_bounding_box");
    return 1;
  }
  // ///////////////////////////////////////////////////////////
  // //in order to work with the rect structure from openCV convention
  // std::cout<<"-------yoloBoxCv---------------"<<std::endl;
  // yoloBoxCv.x=yoloData.topleft_x;
  // yoloBoxCv.y=yoloData.topleft_y;
  // yoloBoxCv.width=yoloData.bottomright_x-yoloData.topleft_x;
  // yoloBoxCv.height=yoloData.bottomright_y-yoloData.topleft_y;
  // cout<<yoloBoxCv<<endl;
  // // cout<<"yoloBoxCv.height:"<<yoloBoxCv.height<<endl;
  // // cout<<"yoloBoxCv.width:"<<yoloBoxCv.width<<endl;
  // // cout<<"yoloBoxCv.x:"<<yoloBoxCv.x<<endl;
  // // cout<<"yoloBoxCv.y:"<<yoloBoxCv.y<<endl;
  // std::cout<<"-------------------------"<<std::endl;
  //////////////////////////////////////////////

  // //CENTER OF THE RECTANGLE FIGURE
  //
  // int x_centerRECTANGLE=yoloBoxCv.x+yoloBoxCv.width/2;
  // int y_centerRECTANGLE=yoloBoxCv.y+yoloBoxCv.height /2;
  // std::cout<<"x_centerRECT: "<<x_centerRECTANGLE<<"\n "<<"y_centerRECT: "<<y_centerRECTANGLE<<endl;
  //
  // int minValue=std::min(yoloBoxCv.height,yoloBoxCv.width);
  // std::cout<<"minValue:"<<minValue<<std::endl;
  // int tmp_1=yoloBoxCv.x+minValue;
  // int tmp_2=yoloBoxCv.y+minValue;
  //
  // //New computation for the square bounding_box
  // yoloBoxCv.width=tmp_1-yoloBoxCv.x;
  // yoloBoxCv.height=tmp_2-yoloBoxCv.y;
  // cout<<"yoloBoxCv.height Square:"<<yoloBoxCv.height<<endl;
  // cout<<"yoloBoxCv.width Square:"<<yoloBoxCv.width<<endl;
  // //CENTER OF THE SQUARE FIGURE
  // int x_centerSQUARE=yoloBoxCv.x+yoloBoxCv.width/2;
  // int y_centerSQUARE=yoloBoxCv.y+yoloBoxCv.height /2;
  // std::cout<<"x_centerSQUARE: "<<x_centerSQUARE<<"\n "<<"y_centerSQUARE: "<<y_centerSQUARE<<endl;
  //
  // int correctionX=x_centerRECTANGLE-x_centerSQUARE;
  // int correctionY=y_centerRECTANGLE-y_centerSQUARE;
  //
  // std::cout<<"correctionX: "<<correctionX<<"\n "<<"correctionY: "<<correctionY<<endl;
  // yoloBoxCvSQUARE.x=yoloBoxCv.x+correctionX;
  // yoloBoxCvSQUARE.y=yoloBoxCv.y+correctionY;
  // yoloBoxCvSQUARE.width=yoloBoxCv.width;
  // yoloBoxCvSQUARE.height=yoloBoxCv.width;
  // std::cout<<"yoloBoxCvSQUARE"<<yoloBoxCvSQUARE<<std::endl;
//////////////////////////////////////////////////////////////

cout<<"Something much more nicer!!!"<<endl;
cv::Rect yoloBoxCvSQUARE;
cv::Rect yoloBoxCvRECTANGLE;
geometry_msgs::Point centerRECTANGLE;
geometry_msgs::Point centerSQUARE;
geometry_msgs::Point correctionDISTANCE;
geometry_msgs::Point tmp_WH;
geometry_msgs::Point auxVALUE;
int lowValue;
std::vector<cv::Rect> vectBoundingBoxes;





yoloBoxCvRECTANGLE.x=yoloData.topleft_x;
yoloBoxCvRECTANGLE.y=yoloData.topleft_y;
yoloBoxCvRECTANGLE.width=yoloData.bottomright_x-yoloData.topleft_x;
yoloBoxCvRECTANGLE.height=yoloData.bottomright_y-yoloData.topleft_y;
// cout<<"yoloBoxCvRECTANGLE.height:"<<yoloBoxCvRECTANGLE.height<<endl;
// cout<<"yoloBoxCvRECTANGLE.width:"<<yoloBoxCvRECTANGLE.width<<endl;
std::cout<<"-------yoloBoxCvRECTANGLE---------------"<<std::endl;
cout<<yoloBoxCvRECTANGLE<<endl;
std::cout<<"-------------------------"<<std::endl;
std::cout<<"-------------------------"<<std::endl;

centerRECTANGLE.x=yoloBoxCvRECTANGLE.x+yoloBoxCvRECTANGLE.width/2;
centerRECTANGLE.y=yoloBoxCvRECTANGLE.y+yoloBoxCvRECTANGLE.height /2;
// std::cout<<"centerRECTANGLE.x: "<<centerRECTANGLE.x<<endl;
// std::cout<<"centerRECTANGLE.y: "<<centerRECTANGLE.y<<endl;


//Computing the lowest values between width and hight given by the yolo detector
lowValue=std::min(yoloBoxCvRECTANGLE.height,yoloBoxCvRECTANGLE.width);
std::cout<<"lowValue: "<<lowValue<<std::endl;

//Adding the minValue to the topleft point
tmp_WH.x=yoloBoxCvRECTANGLE.x+lowValue;
tmp_WH.y=yoloBoxCvRECTANGLE.y+lowValue;

//New computation for the square bounding_box
yoloBoxCvSQUARE.width=tmp_WH.x-yoloBoxCvRECTANGLE.x;
yoloBoxCvSQUARE.height=tmp_WH.y-yoloBoxCvRECTANGLE.y;

yoloBoxCvSQUARE.x=yoloData.topleft_x;
yoloBoxCvSQUARE.y=yoloData.topleft_y;

//CENTER OF THE SQUARE FIGURE
centerSQUARE.x=yoloBoxCvSQUARE.x+yoloBoxCvSQUARE.width/2;
centerSQUARE.y=yoloBoxCvSQUARE.y+yoloBoxCvSQUARE.height /2;
std::cout<<"centerSQUARE: "<<centerSQUARE.x<<endl;
std::cout<<"centerSQUARE: "<<centerSQUARE.y<<endl;


//DISPLACEMENT OF THE SQUARE CENTER INTO THE RECTANGLE CENTER
correctionDISTANCE.x=abs(centerRECTANGLE.x-centerSQUARE.x);
correctionDISTANCE.y=abs(centerRECTANGLE.y-centerSQUARE.y);
std::cout<<"correctionDISTANCE.x: "<<correctionDISTANCE.x<<endl;
std::cout<<"correctionDISTANCE.y: "<<correctionDISTANCE.y<<endl;
yoloBoxCvSQUARE.x=yoloBoxCvSQUARE.x+correctionDISTANCE.x;
yoloBoxCvSQUARE.y=yoloBoxCvSQUARE.y+correctionDISTANCE.y;

std::cout<<"-------yoloBoxCvSQUARE---------------"<<std::endl;
cout<<yoloBoxCvSQUARE<<endl;
std::cout<<"-------------------------"<<std::endl;


vectBoundingBoxes.push_back(yoloBoxCvRECTANGLE);
vectBoundingBoxes.push_back(yoloBoxCvSQUARE);

for (std::vector<cv::Rect>::iterator it = vectBoundingBoxes.begin(); it != vectBoundingBoxes.end(); ++it)
  std::cout << ' ' << *it;
std::cout << '\n';

std::vector<int> vect;

vect.push_back(10);
vect.push_back(20);
vect.push_back(30);


// if( image.empty() )                      // Check for invalid input
// {
//     cout <<  "Could not open or find the image" << std::endl ;
//     return -1;
// }
 Mat outImg;
 //width=640 and  height=480 because of yolo detector, i cannot plot the
 //bounding box on the original image. It would be stupid thing to do.
namedWindow( "Display window", WINDOW_AUTOSIZE ); // Create a window for display.
cv::resize(image, outImg, cv::Size(640, 480));
cv::rectangle(outImg, yoloBoxCvSQUARE.tl(), yoloBoxCvSQUARE.br(), cv::Scalar(100, 100, 200), 2, CV_AA);
imshow( "Display window", outImg);                // Show our image inside it.
waitKey(0); // Wait for a keystroke in the window
//waitKey(2000);



// for (std::vector<int>::iterator it = vect.begin(); it != vect.end(); ++it)
//   std::cout << ' ' << *it;
// std::cout << '\n';

// cout<<"yoloBoxCvSQUARE.x:"<<yoloBoxCvSQUARE.x<<endl;
// cout<<"yoloBoxCvSQUARE.y:"<<yoloBoxCvSQUARE.y<<endl;
// cout<<"yoloBoxCvSQUARE.height:"<<yoloBoxCvSQUARE.hight<<endl;
// cout<<"yoloBoxCvSQUARE.width:"<<yoloBoxCvSQUARE.width<<endl;

  return 0;
}



// CPP program to create an empty vector
// and one by one push values.





//std::cout<<"srvYolo.response.bounding_box: \n"<<srvYolo.response.bounding_box_tmp<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.confidence<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.topleft_x<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.topleft_y<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.bottomright_x<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.bottomright_y<<std::endl;
// std::cout<<srvYolo.response.bounding_box_tmp.Class<<std::endl;
