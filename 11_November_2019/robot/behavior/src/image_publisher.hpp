#ifdef IMAGE_PUBLISHER
#ifndef IMAGE_PUBLISHER_HPP
#define IMAGE_PUBLISHER_HPP

#include "msgs/messages.pb.h"
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <opencv2/opencv.hpp>
#include <string>

class ImagePublisher {
private:
  ignition::transport::Node node;
  std::string topic;
  decltype(node.Advertise<ImageMsg>(topic)) pub;

public:
  ImagePublisher();
  void publish(cv::Mat image);
};

#endif
#endif
