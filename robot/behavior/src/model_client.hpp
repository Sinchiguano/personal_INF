#ifndef MODEL_CLIENT_HPP
#define MODEL_CLIENT_HPP

#include <iostream>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <zmq.hpp>

#include <convert_3d.hpp>
#include <utilities.hpp>

class ModelClient {
public:
  zmq::context_t context;
  zmq::socket_t socket;
  int port;

  ModelClient(int port);
  std::string str(zmq::message_t &m);
  std::string get();
  void send(std::string const &msg);
  void end();
  ~ModelClient() {}
};

std::vector<double> get_occlusion_via_server(cv::Mat img, cv::Rect const &box,
                                             ModelClient &m);
std::vector<double> get_axis_translation_via_server(cv::Mat img,
                                                    cv::Rect const &box,
                                                    ModelClient &m);

GeometricTransformation
get_object_tfm_wrt_base(cv::Mat img, cv::Rect const &box, ModelClient &m,
                        GeometricTransformation const &robot_tfm);
#endif // !MODEL_CLIENT_HPP
