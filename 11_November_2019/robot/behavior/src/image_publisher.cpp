#ifdef IMAGE_PUBLISHER
#include "msgs/messages.pb.h"
#include <chrono>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "image_publisher.hpp"

  ImagePublisher::ImagePublisher()
      : node{}, topic{"/image"}, pub{node.Advertise<ImageMsg>(topic)} {
    // cv::namedWindow("bpwindow", CV_WINDOW_NORMAL);
    if (!pub) {
      std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
    }
  }

  void ImagePublisher::publish(cv::Mat image) {
    std::vector<uchar> buff;
    // cv::imshow("bpwindow", image);
    cv::imencode(".jpg", image, buff);
    ImageMsg msg;
    msg.set_width(image.cols);
    msg.set_height(image.rows);
    msg.set_size(buff.size());
    msg.set_data(&buff[0], buff.size());
    msg.set_id(0);
    while (true) {
      auto res = pub.Publish(msg);
      //std::cout << "res " << res << std::endl;
      //std::cin.get();
      // cv::waitKey(0);
    }
  }

#endif
