/*
 * File name: main.cc
 * Date:      2018-03-22 
 * Author:    Miroslav Kulich
 */

/*
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/imgcodecs/imgcodecs.hpp"
#include <vector>
#include "binloc.h"


// using namespace cv;




int main(int argc, char **argv) {
  // Declare the output variables
  const char *default_file = "../right_close.jpg";
  const char *filename = argc >= 2 ? argv[1] : default_file;

  // Load an image
  // cv::Mat src = cv::imread(filename, cv::IMREAD_GRAYSCALE);
  cv::Mat src = cv::imread(filename, 0); //TODO grayscale
  // Check if image is loaded fine
  if (src.empty()) {
    printf(" Error opening image\n");
    printf(" Program Arguments: [image_name -- default %s] \n", default_file);
    return -1;
  }

  cv::Point result;
  detectCorner(src, imr::RIGHT_CLOSE, result);


  // demo of bin localization
  // initial guess of bin location (not needed to be precise)
  imr::PointVec bin = { cv::Point2d(-5,-2), cv::Point2d(5,-2), cv::Point2d(5,2), cv::Point2d(-5,2) };
  
  // simulation of detected corners
  imr::Transform ch(0.3, 0.4, 0.2);
  imr::PointVec cor = imr::transform(bin,ch);
  cor[2].x += 0.1; // introducing an error to one measurement

  // get location of the bin
  imr::PointVec pos = imr::localizeBin(cor);

  // print measured and fitted corners
  for(int i=0; i< bin.size(); i++) {
    std::cout << cor[i].x << " " << cor[i].y << "  <->  " << pos[i].x << " " << pos[i].y << std::endl;
  }


  return 0;
}
*/