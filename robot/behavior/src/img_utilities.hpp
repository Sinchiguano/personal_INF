#ifndef IMAGE_UTILITIES_HPP
#define IMAGE_UTILITIES_HPP

#include <vector>
#include <opencv2/opencv.hpp>

// not now    std::vector<cv::Rect> get_bounding_boxes (
// not now            std::vector<std::vector<cv::Point>> const &msers);

cv::Mat draw_boxes(cv::Mat const &img, std::vector<cv::Rect> const &boxes,
        cv::Scalar const &color = cv::Scalar(0, 0, 255),
        int linewidth = 1);

cv::Mat draw_boxes(cv::Mat const &img, std::vector<cv::RotatedRect> const &boxes,
        cv::Scalar const &color = cv::Scalar(255, 0, 255),
        int linewidth = 1);

// not now    cv::Mat draw_regions(cv::Mat img, std::vector<std::vector<cv::Point>> regions, bool random_colors = true, cv::Scalar color = cv::Scalar(0,0,255));

cv::Rect get_region_boundary(std::vector<cv::Point> const &region);

cv::Mat crop_rotated(cv::Mat img, cv::RotatedRect const &box);

cv::RotatedRect get_bigger_box(cv::Rect const &box, double scale);
cv::Rect intersection(cv::Rect const &rect, cv::Mat img);

int area(cv::Rect const &box);

bool area_cmp(cv::Rect const &a, cv::Rect const &b);

#endif /* ifndef IMAGE_UTILS_HPP */
