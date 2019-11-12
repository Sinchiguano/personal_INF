/*
 * File name: binloc.cc
 * Date:      2018-03-22 
 * Author:    Miroslav Kulich
 */

#include <iostream>

#include "binloc.h"

#include "opencv2/highgui/highgui.hpp"
// #include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "CannyLine.h"

using namespace cv;
using namespace std;
using namespace imr;



void drawLine(Mat &dst, float rho, float theta, Scalar color) {
  Point pt1, pt2;
  double a = cos(theta), b = sin(theta);
  double x0 = a * rho, y0 = b * rho;
  pt1.x = cvRound(x0 + 10000 * (-b));
  pt1.y = cvRound(y0 + 10000 * (a));
  pt2.x = cvRound(x0 - 10000 * (-b));
  pt2.y = cvRound(y0 - 10000 * (a));
  line(dst, pt1, pt2, color, 2); //, CV_AA);
}

void polarToPoints(float rho, float theta, Point &pt1, Point &pt2) {
  double a = cos(theta), b = sin(theta);
  double x0 = a * rho, y0 = b * rho;
  pt1.x = cvRound(x0 + 10000 * (-b));
  pt1.y = cvRound(y0 + 10000 * (a));
  pt2.x = cvRound(x0 - 10000 * (-b));
  pt2.y = cvRound(y0 - 10000 * (a));
}

/// ----------------------------------------------------------------------------
/// @brief segmentIntersection based on algorithm SegSegInt from Computation
/// Geomery in C
///
/// @param a segment(a,b)
/// @param b
/// @param c segment(c,d)
/// @param d
/// @param p interseciton point
///
/// @return '1' proper intersection, '0' no intersection
///         'e' edge intersection, 'v' vertex intersection
/// ----------------------------------------------------------------------------


char segmentIntersection(const Point &a, const Point &b, const Point &c, const Point &d, Point &p) {
  static const double EPS = 0.00001;
  double s, t;
  double num, denom;
  char code = '0';

  denom = a.x * (d.y - c.y) + b.x * (c.y - d.y) + d.x * (b.y - a.y) +
          c.x * (a.y - b.y);

  if (fabs(denom) < EPS) {
    code = 'p'; //this shoulh never happen in our case
  } else {
    num = a.x * (d.y - c.y) + c.x * (a.y - d.y) + d.x * (c.y - a.y);
    // if ( (num == 0.0) || (num == denom))
    if ((fabs(num) < EPS) || (fabs(num - denom) < EPS)) {
      code = 'v';
    }
    s = num / denom;
    num = -(a.x * (c.y - b.y) + b.x * (a.y - c.y) + c.x * (b.y - a.y));
    //                    if ( (num == 0.0) || (num == denom) )
    if ((fabs(num) < EPS) || (fabs(num - denom) < EPS)) {
      code = 'v';
    }
    t = num / denom;

    if ((0.0 < s) && (s < 1.0) && (0.0 < t) && (t < 1.0)) {
      code = '1';
    } else if (((0.0 > s) || (s > 1.0)) && ((0.0 > t) || (t > 1.0))) {
      code = '0';
    }
    p.x = a.x + s * (b.x - a.x);
    p.y = a.y + s * (b.y - a.y);
  }
  return code;
}

char intersection(float rho1, float theta1, float rho2, float theta2, Point &p) {
  Point a,b,c,d;

  polarToPoints(rho1, theta1, a, b);
  polarToPoints(rho2, theta2, c, d);  
  return segmentIntersection(a,b,c,d,p);
}


void imr::detectCorner(Mat &img, Corner corner, Point &p) {
  Mat cdst;
  GaussianBlur(img, img, Size(3, 3), 0, 0);

// detect lines in the image
  CannyLine detector;
  std::vector<std::vector<float>> lines;
  detector.cannyLine(img, lines);

//draw them into a new image
  cv::Mat dst(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
  for (int m = 0; m < lines.size(); ++m) {
    cv::line(dst, cv::Point(lines[m][0], lines[m][1]),
             cv::Point(lines[m][2], lines[m][3]), 255, 1); //, CV_AA );
  }

// find lines making use Hough transform
  vector<Vec2f> hlines;
  HoughLines(dst, hlines, 1, CV_PI / 360, 200);


  float left = 10000;
  float right = 0;
  float top = 10000;
  float bottom = 0;
  int idLeft, idRight, idTop, idBottom;

  for (size_t i = 0; i < hlines.size(); i++) {
    float rho = hlines[i][0];
    float theta = hlines[i][1];
    if (fabs(cos(theta)) > 0.9) {
      if (fabs(left) > fabs(rho)) {
        left = rho;
        idLeft = i;
      }
      if (fabs(right) < fabs(rho)) {
        right = rho;
        idRight = i;
      }
    }
    if (fabs(cos(theta)) < 0.1) {
      if (fabs(top) > fabs(rho)) {
        top = rho;
        idTop = i;
      }
      if (fabs(bottom) < fabs(rho)) {
        bottom = rho;
        idBottom = i;
      }
    }
  }

  cvtColor(img, cdst, COLOR_GRAY2BGR);

  for (int m = 0; m < lines.size(); ++m) {
    cv::line(cdst, cv::Point(lines[m][0], lines[m][1]),
             cv::Point(lines[m][2], lines[m][3]), Scalar(255, 255, 255),
             2);
  }


  char r;
  switch (corner) { 
    case RIGHT_FAR: 
        r = intersection(hlines[idLeft][0], hlines[idLeft][1], hlines[idBottom][0], hlines[idBottom][1], p);
        drawLine(cdst, hlines[idLeft][0], hlines[idLeft][1], Scalar(0, 255, 0));
        drawLine(cdst, hlines[idBottom][0], hlines[idBottom][1], Scalar(0, 255, 0));
    break;
    case LEFT_FAR: 
        r = intersection(hlines[idRight][0], hlines[idRight][1], hlines[idBottom][0], hlines[idBottom][1], p);
        drawLine(cdst, hlines[idRight][0], hlines[idRight][1], Scalar(0, 255, 0));
        drawLine(cdst, hlines[idBottom][0], hlines[idBottom][1], Scalar(0, 255, 0));
    break;
    case LEFT_CLOSE: 
        r = intersection(hlines[idRight][0], hlines[idRight][1], hlines[idTop][0], hlines[idTop][1], p);
        drawLine(cdst, hlines[idRight][0], hlines[idRight][1], Scalar(0, 255, 0));
        drawLine(cdst, hlines[idTop][0], hlines[idTop][1], Scalar(0, 255, 0));
    break;
      
    case RIGHT_CLOSE: 
        r = intersection(hlines[idLeft][0], hlines[idLeft][1], hlines[idTop][0], hlines[idTop][1], p);
        drawLine(cdst, hlines[idLeft][0], hlines[idLeft][1], Scalar(0, 255, 0));
        drawLine(cdst, hlines[idTop][0], hlines[idTop][1], Scalar(0, 255, 0));

    break;
  }

  cv::circle(cdst, p, 10, Scalar(0,0,255),4);
  imwrite("result_" + cornerString[corner] +".png", cdst);
}

Point2d getMean (PointVec const &rp) {
    Point2d mean;
    for (auto const & point : rp) {
        mean.x += point.x;
        mean.y += point.y;
    }
    mean.x /= rp.size();
    mean.y /= rp.size();
    return mean;
}



Transform getTransform(PointVec corners, PointVec bin) {
  Transform result;
  Point2d meanCor = getMean(corners);
  Point2d meanBin = getMean(bin);

//  compute rotation
    double sum1{0}, sum2{0}, sum3{0}, sum4{0};
    for (int i = 0; i < corners.size(); i++) {
        // numerator
        sum1 += (bin[i].x - meanBin.x)*(corners[i].y - meanCor.y);
        sum2 += (bin[i].y - meanBin.y)*(corners[i].x - meanCor.x);
        // denomenator
        sum3 += (bin[i].x - meanBin.x)*(corners[i].x - meanCor.x);
        sum4 += (bin[i].y - meanBin.y)*(corners[i].y - meanCor.y);
    }


    result.omega = std::atan2((sum1-sum2),(sum3+sum4));

//  compute translation
    result.tx = meanCor.x - (meanBin.x * std::cos(result.omega) - meanBin.y * std::sin(result.omega));
    result.ty = meanCor.y - (meanBin.x * std::sin(result.omega) + meanBin.y * std::cos(result.omega));
    return result;
}

PointVec imr::transform(PointVec &points, Transform &t) {
  PointVec result;
  Point2d p;
  for(int i=0;i<points.size();i++) {
    p.x = cos(t.omega)*points[i].x - sin(t.omega)*points[i].y + t.tx;
    p.y = sin(t.omega)*points[i].x + cos(t.omega)*points[i].y + t.ty;
    result.push_back(p);
  }
  return result;
}

double getError(PointVec a, PointVec b) {
  double result = 0;
  for(int i=0;i<a.size();i++) {
    result += (a[i].x - b[i].x)*(a[i].x - b[i].x) + (a[i].y - b[i].y)*(a[i].y - b[i].y);
  }
  return result;
}

PointVec imr::localizeBin(PointVec cor) {
  static PointVec bin = { Point2d(-5,-2), Point2d(5,-2), Point2d(5,2), Point2d(-5,2) }; //TODO: enter real coordinates
  Transform bestT;
  double minError = std::numeric_limits<double>::max();

  for(int i=0; i<cor.size(); i++) {
    PointVec binT;
    PointVec corT;
    for(int j=0; j<cor.size(); j++) {
      if (i!=j) {
        binT.push_back(bin[j]);
        corT.push_back(cor[j]);
      }
    }

    Transform t = getTransform(corT, binT);
    PointVec pos = transform(binT,t);
    double err = getError(pos, corT);
    std::cout << i << "  tx: " << t.tx << " ty: " << t.ty << " omega: " << t.omega  << "     error: " << err << std::endl;
    if (minError > err) {
      minError = err;
      bestT = t; 
    }
  }

  return transform(bin,bestT);
}
