/*
 * File name: binloc.h
 * Date:      2018-03-22 
 * Author:    Miroslav Kulich
 */


#ifndef IMR_BIN_LOC
#define IMR_BIN_LOC

#include "opencv2/imgproc/imgproc.hpp"

namespace imr {

/// type of the corner
enum Corner { RIGHT_FAR, LEFT_FAR, RIGHT_CLOSE, LEFT_CLOSE };

/// string describing corner type
const std::string cornerString[] = {"right_far", "left_far", "right_close", "left_close" }; 

struct Transform {
  double tx;
  double ty;
  double omega;
  Transform(double tx, double ty, double omega) : tx(tx), ty(ty), omega(omega) {};
  Transform() {};
};

typedef std::vector<cv::Point2d> PointVec; 


/// ----------------------------------------------------------------------------
/// @brief finds an inner corner of the bin on the image
///
/// @param img      input image
/// @param corner   type of the corner
/// @param p interseciton point
///
/// ----------------------------------------------------------------------------

void detectCorner(cv::Mat &img, Corner corner, cv::Point &p); 



/// ----------------------------------------------------------------------------
/// @brief returns real coordinates of a bin
///
/// @param cor      real coordinates of corners 
///
/// ----------------------------------------------------------------------------

PointVec localizeBin(PointVec cor);

/// ----------------------------------------------------------------------------
/// @brief transform points
///
/// @param points  points to be transformed
/// @param t transformation to be used
/// @result transformed points
///
/// ----------------------------------------------------------------------------

PointVec transform(PointVec &points, Transform &t);

};

#endif