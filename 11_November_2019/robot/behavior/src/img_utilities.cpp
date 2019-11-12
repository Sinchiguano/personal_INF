#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <random>

#include "img_utilities.hpp"

std::vector<cv::Rect>
get_bounding_boxes(std::vector<std::vector<cv::Point>> const &msers) {
  std::vector<cv::Rect> boxes;
  boxes.reserve(msers.size());
  std::transform(msers.begin(), msers.end(), std::back_inserter(boxes),
                 [](decltype(*msers.begin()) region) {
                   return get_region_boundary(region);
                 });
  return boxes;
}

cv::Mat draw_boxes(cv::Mat const &img, std::vector<cv::Rect> const &boxes,
                   cv::Scalar const &color, int linewidth) {
  auto img_copy = img.clone();
  for (auto const &box : boxes) {
    cv::rectangle(img_copy, box, color, linewidth);
  }
  return img_copy;
}

cv::Mat draw_boxes(cv::Mat const &img,
                   std::vector<cv::RotatedRect> const &boxes,
                   cv::Scalar const &color, int linewidth) {
  auto img_copy = img.clone();
  for (auto const &box : boxes) {
    cv::Point2f rect_points[4];
    box.points(rect_points);
    for (int j = 0; j < 4; j++) {
      cv::line(img_copy, rect_points[j], rect_points[(j + 1) % 4], color,
               linewidth);
    }
  }
  return img_copy;
}

cv::Rect intersection(cv::Rect const &rect, cv::Mat img) {
  cv::Rect img_rect(0, 0, img.size().width, img.size().height);
  return img_rect & rect;
}

std::tuple<cv::Mat, cv::RotatedRect>
remove_non_roi_for_rotated_crop(cv::Mat src,
                                cv::RotatedRect const &box_wrt_src) {
  auto box_more = box_wrt_src;
  box_more.size.width *= 1.3;
  box_more.size.height *= 1.3;
  auto bounding_box = box_more.boundingRect();
  bounding_box = intersection(bounding_box, src);
  cv::Mat img(src, bounding_box);

  auto box = box_wrt_src;
  box.center.x -= bounding_box.x;
  box.center.y -= bounding_box.y;

  return std::make_tuple(img, box);
}

cv::Mat crop_rotated(cv::Mat src, cv::RotatedRect const &box_wrt_src) {
  // remove area around the box to reduce CPU time
  auto around_roi = remove_non_roi_for_rotated_crop(src, box_wrt_src);
  auto img = std::get<0>(around_roi);
  auto box = std::get<1>(around_roi);

  // matrices we'll use
  cv::Mat M, rotated, cropped;
  // get angle and size from the bounding box
  float angle = box.angle;
  cv::Size rect_size = box.size;
  // get the rotation matrix
  M = getRotationMatrix2D(box.center, angle, 1.0);
  // perform the affine transformation
  cv::warpAffine(img, rotated, M, img.size(), cv::INTER_CUBIC);
  // crop the resulting image
  cv::getRectSubPix(rotated, rect_size, box.center, cropped);

  return cropped;
}

cv::RotatedRect get_bigger_box(cv::Rect const &box, double scale) {
  auto rot_box = cv::RotatedRect(
      cv::Point(box.x + 0.5 * box.width, box.y + 0.5 * box.height),
      cv::Size(scale * box.width, scale * box.height), 0);
  return rot_box;
}

cv::Rect get_region_boundary(std::vector<cv::Point> const &region) {
  int min_x{std::numeric_limits<int>::max()},
      min_y{std::numeric_limits<int>::max()}, max_x{0}, max_y{0};
  for (auto const &pt : region) {
    min_x = std::min(pt.x, min_x);
    min_y = std::min(pt.y, min_y);
    max_x = std::max(pt.x, max_x);
    max_y = std::max(pt.y, max_y);
  }
  return cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y);
}

int area(cv::Rect const &box) { return box.width * box.height; }

bool area_cmp(cv::Rect const &a, cv::Rect const &b) {
  return area(a) < area(b);
}
