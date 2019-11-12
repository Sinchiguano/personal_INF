#include "model_client.hpp"
#include "img_utilities.hpp"
#include "app_log.hpp"

ModelClient::ModelClient(int port)
    : context(1), socket(context, ZMQ_REQ), port(port) {
  socket.connect("tcp://localhost:" + Ttos(port));
}

std::string ModelClient::str(zmq::message_t &m) {
  return std::string(static_cast<char *>(m.data()), m.size());
}

std::string ModelClient::get() {
  zmq::message_t request(3);
  memcpy(request.data(), "get", 3);
  socket.send(request);

  zmq::message_t reply;
  std::cout << "waiting for response..." << std::endl;
  socket.recv(&reply);

  return str(reply);
}

void ModelClient::send(std::string const &msg) {
  zmq::message_t request(msg.size());
  memcpy(request.data(), msg.c_str(), msg.size());
  socket.send(request);

  zmq::message_t reply;
  socket.recv(&reply);
}

void ModelClient::end() {
  zmq::message_t request(3);
  memcpy(request.data(), "end", 3);
  socket.send(request);

  zmq::message_t reply;
  socket.recv(&reply);
}

std::vector<double> get_occlusion_via_server(cv::Mat img, cv::Rect const &box,
                                             ModelClient &m) {
  cv::Mat cropped = crop_rotated(img, get_bigger_box(box, 1.5));
  cv::imwrite("/home/binpicking/detect_piece/pose_estimation/"
              "occlusion_big/server/runtime_data/input.jpg",
              cropped);

  auto responce = m.get();
  double occl, unoccl, unclear;
  std::stringstream ss(responce);
  ss >> occl;
  ss >> unoccl;
  ss >> unclear;

  return std::vector<double>{occl, unoccl, unclear};
}

std::vector<double> get_axis_translation_via_server(cv::Mat img,
                                                    cv::Rect const &box,
                                                    ModelClient &m) {
  cv::Mat cropped(cv::Mat(img, box));
  cv::imwrite("/home/binpicking/detect_piece/pose_estimation/"
              "pose/server/runtime_data/input.jpg",
              cropped);

  auto responce = m.get();
  //INFO("axis translation responce ", responce);
  double rx, ry, rz;
  std::stringstream ss(responce);
  ss >> rx;
  ss >> ry;
  ss >> rz;
  rx *= -1;
  ry *= -1;
  rz *= -1;

  double x_bo_p, y_bo_p, z_rel;
  ss >> x_bo_p;
  ss >> y_bo_p;
  ss >> z_rel;

  double f = 3820.5;
  //double img_center_x = 2592 / 2, img_center_y = 1944 / 2;
  double img_center_x = 2592 / 4, img_center_y = 1944 / 4;
  //double z = z_rel * 1000.0 / box.width - 0.083;
  double z = z_rel * 500.0 / box.width - 0.083;
  double x = (img_center_x - (box.x + x_bo_p * box.width)) * (z + 0.083) / f;
  double y = (img_center_y - (box.y + y_bo_p * box.width)) * (z + 0.083) / f;


  //Ignore xy from neural network and use this instead???
  //in_box_ratio = [0.62, 0.44]
  //x = (img_center_x - (box.x + 0.62 * box.width)) * (z + 0.083) / f;
  //y = (img_center_y - (box.y + 0.44 * box.width)) * (z + 0.083) / f;


  return std::vector<double>{rx, ry, rz, x, y, z};
}

GeometricTransformation
get_object_tfm_wrt_base(cv::Mat img, cv::Rect const &box, ModelClient &m,
                        GeometricTransformation const &robot_tfm) {
  auto axis_tr = get_axis_translation_via_server(img, box, m);
  Vector3D axis(axis_tr[0], axis_tr[1], axis_tr[2]);
  Vector3D xyz(axis_tr[3], axis_tr[4], axis_tr[5]);

  //INFO("axis\n", axis);
  RotationMatrix rot = get_rotation(Vector3D(1, 0, 0), axis);

  // iter 3
  GeometricTransformation T_ro = translation(xyz) * rotation(rot);
  //INFO("T_ro");
  //INFO(T_ro);
  GeometricTransformation T_bo = robot_tfm * T_ro;
  return T_bo;
}