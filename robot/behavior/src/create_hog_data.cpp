#include <string>
#include <boost/optional.hpp>
#include <yaml-cpp/yaml.h>
#include "app_log.hpp"
#include "convert_3d.hpp"
#include "hog_detector.hpp"
#include "img_utilities.hpp"



#include "YoloClient.h"


// C++ program to demonstrate the use of std::min
#include <algorithm>
#include "geometry_msgs/Point.h"

using namespace std;
namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;
using namespace sensor_msgs;

RobotPose toRobotPose(YAML::Node const &n)
{
  RobotPose robot_pose;
  robot_pose.x = n[0].as<double>();
  robot_pose.y = n[1].as<double>();
  robot_pose.z = n[2].as<double>();
  robot_pose.rx = n[3].as<double>();
  robot_pose.ry = n[4].as<double>();
  robot_pose.rz = n[5].as<double>();
  INFO("toRobotPose: ok");
  return robot_pose;
}

// data_dir : dir with files and config
// robot_data_fname : config filename in data_dir
RobotData readRobotData(std::string const &data_dir, std::string const &robot_data_fname) 
{
  RobotData robot_data;
  auto robot_data_node = YAML::LoadFile(data_dir + robot_data_fname);
  auto common_node = robot_data_node["common"];
  INFO("get common_node: ok");
  auto robot_on_object_pose_node = common_node["robot_on_object_pose"];
  INFO("get robot_on_object_pose_node: ok");
  auto camera_f = common_node["camera_f"];
  INFO("get camera_f: ok");
  auto camera_offset_z = common_node["camera_offset_z"];
  INFO("get camera_offset_z: ok");
  auto distance_sonar = common_node["distance_sonar"];
  INFO("get distance_sonar: ok");
  auto list_node = robot_data_node["list"];
  INFO("get list_node: ok");

  RobotData::Common common;
  common.robot_on_object_pose = toRobotPose(robot_on_object_pose_node);
  common.camera_f = camera_f.as<double>();
  INFO("get camera_f: ok");
  common.camera_offset_z = camera_offset_z.as<double>();
  INFO("get camera_offset_z: ok");
  common.distance = distance_sonar.as<int>();
  INFO("get distance_sonar: ok");
  robot_data.common = common;

  RobotData::List list;
  std::transform(list_node.begin(), list_node.end(), std::back_inserter(list),
                 [&data_dir](YAML::Node const &element_node) {
                   RobotData::ListElement element;
                   element.robot_pose = toRobotPose(element_node["robot_pose"]);
                   element.camera_img_fname =
                       data_dir + element_node["camera_img_fname"].as<std::string>();
                   return element;
                 });
  robot_data.list = list;
  return robot_data;
}

YAML::Node toYAML(RobotData const &robot_data) 
{
  YAML::Node main;

  {
    YAML::Node common;
    auto rp = robot_data.common.robot_on_object_pose;
    common["robot_on_object_pose"] =
        std::vector<double>{rp.x, rp.y, rp.z, rp.rx, rp.ry, rp.rz};
    common["camera_f"] = robot_data.common.camera_f;  
    common["camera_offset_z"] = robot_data.common.camera_offset_z;
    if(robot_data.common.distance) {
      common["distance_sonar"] = *robot_data.common.distance;
    } else {
      common["distance_sonar"] = 0;
    }
    main["common"] = common;
  }

  YAML::Node list;
  for (auto elem : robot_data.list) {
    YAML::Node elem_n;
    auto rp = *elem.robot_pose;
    elem_n["robot_pose"] =
        std::vector<double>{rp.x, rp.y, rp.z, rp.rx, rp.ry, rp.rz};
    elem_n["camera_img_fname"] = elem.camera_img_fname;
    elem_n["distance_sonar"] = elem.distance;
    list.push_back(elem_n);
  }
  main["list"] = list;
  return main;
}
void create_hog_data() 
{
  //std::cout<<"Something much more nicer!!!"<<std::endl;
  cv::Rect yoloBoxCvSQUARE;
  cv::Rect yoloBoxCvRECTANGLE;
  geometry_msgs::Point centerRECTANGLE;
  geometry_msgs::Point centerSQUARE;
  geometry_msgs::Point correctionDISTANCE;
  geometry_msgs::Point tmp_WH;
  geometry_msgs::Point auxVALUE;
  std::vector<cv::Rect> vectBoundingboxes;
  int lowValue;
  cv::Rect yoloBoxCv;
  Mat outImg;
  ros::NodeHandle n;
  ros::ServiceClient clientYoloTmp = n.serviceClient<yolo_detector::YoloDetection>("bounding_box_given_image");
  yolo_detector::YoloDetection srvYoloTmp;
  yolo_detector::BoundingBox yoloData;

  // std::string input_dir {"/home/yumi/skoda/hog_converter/build/input/1/"};
  // std::string output_dir {"/home/yumi/skoda/hog_converter/build/yolo_data/1"};
  
  int counterFolder=150;

  while(counterFolder<1)
  {

    std::string input_dir {"/home/yumi/skoda/hog_converter/build/input/"+std::to_string(counterFolder)+"/"};
    std::string output_dir {"/home/yumi/skoda/hog_converter/build/yolo_sonar/"+std::to_string(counterFolder)+"/"};
    RobotData robot_data = readRobotData(input_dir, "robot_data.yaml");


    int idx = 0;
    HOGDetectorLogger l(output_dir, &idx);
    HOGDetector d(l);
    auto &list = robot_data.list;

    for (auto &item : list) 
    {
      if (idx % 20 == 0)
        INFO("idx: ", idx, "/", list.size());
      cv::Mat img = cv::imread(item.camera_img_fname);
      cv::resize(img, outImg, cv::Size(640, 480));
      // ///////_YOLO_DETECTOR_///////
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
      srvYoloTmp.request.ImageQuery = *msg;  //ROS_INFO("ImageMsg Send!");

      if (clientYoloTmp.call(srvYoloTmp))
      {
        //std::cout<<srvYolo.response<<std::endl;
        
        //std::cout<<"srvYolo.response.bounding_box: \n"<<srvYolo.response.bounding_box_tmp<<std::endl;
        // yoloData.topleft_x=srvYoloTmp.response.bounding_box_tmp.topleft_x;
        // yoloData.topleft_y=srvYoloTmp.response.bounding_box_tmp.topleft_y;
        // yoloData.bottomright_x=srvYoloTmp.response.bounding_box_tmp.bottomright_x;
        // yoloData.bottomright_y=srvYoloTmp.response.bounding_box_tmp.bottomright_y;
        // yoloData.id=srvYoloTmp.response.bounding_box_tmp.id;
      
        //std::cout<<srvYoloTmp.response.BoundingBoxAnswer<<std::endl;
        yoloData.topleft_x=srvYoloTmp.response.BoundingBoxAnswer.topleft_x;
        yoloData.topleft_y=srvYoloTmp.response.BoundingBoxAnswer.topleft_y;
        yoloData.bottomright_x=srvYoloTmp.response.BoundingBoxAnswer.bottomright_x;
        yoloData.bottomright_y=srvYoloTmp.response.BoundingBoxAnswer.bottomright_y;
        yoloData.id=srvYoloTmp.response.BoundingBoxAnswer.id;
      }
      else
      {
        ROS_ERROR("Failed to call service detect_bounding_box");
      }

      yoloBoxCvRECTANGLE.x=yoloData.topleft_x;
      yoloBoxCvRECTANGLE.y=yoloData.topleft_y;
      yoloBoxCvRECTANGLE.width=yoloData.bottomright_x-yoloData.topleft_x;
      yoloBoxCvRECTANGLE.height=yoloData.bottomright_y-yoloData.topleft_y;
      std::cout<<"-------yoloBoxCvRECTANGLE---------------"<<std::endl;
      cout<<yoloBoxCvRECTANGLE<<endl;
      std::cout<<"-------------------------"<<std::endl;

      centerRECTANGLE.x=yoloBoxCvRECTANGLE.x+yoloBoxCvRECTANGLE.width/2;
      centerRECTANGLE.y=yoloBoxCvRECTANGLE.y+yoloBoxCvRECTANGLE.height /2;

      //Computing the lowest values between width and hight given by the yolo detector
      lowValue=std::min(yoloBoxCvRECTANGLE.height,yoloBoxCvRECTANGLE.width);

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

      //DISPLACEMENT OF THE SQUARE CENTER INTO THE RECTANGLE CENTER
      correctionDISTANCE.x=abs(centerRECTANGLE.x-centerSQUARE.x);
      correctionDISTANCE.y=abs(centerRECTANGLE.y-centerSQUARE.y);

      yoloBoxCvSQUARE.x=yoloBoxCvSQUARE.x+correctionDISTANCE.x;
      yoloBoxCvSQUARE.y=yoloBoxCvSQUARE.y+correctionDISTANCE.y;

      std::cout<<"-------yoloBoxCvSQUARE---------------"<<std::endl;
      cout<<yoloBoxCvSQUARE<<endl;
      std::cout<<"-------------------------"<<std::endl;
      std::vector<cv::Rect> vectBoundingboxes;
      vectBoundingboxes.push_back(yoloBoxCvSQUARE);
      std::cout<<"-------------------------"<<std::endl;
      cout<<"Folder name: "<<output_dir<<endl;

      d.cesar_bypass(outImg,vectBoundingboxes);
      auto target_box_idx = std::distance(vectBoundingboxes.begin(), std::max_element(vectBoundingboxes.begin(), vectBoundingboxes.end(), area_cmp));
      l.add_img_pose_info(robot_data.common, item, target_box_idx,vectBoundingboxes[target_box_idx]);//updated with the bounding box from yolo detector
      namedWindow( "Display window", WINDOW_AUTOSIZE ); // Create a window for display.
      cv::rectangle(outImg, yoloBoxCvSQUARE.tl(), yoloBoxCvSQUARE.br(), cv::Scalar(100, 100, 200), 2, CV_AA);
      imshow( "Display window", outImg);                // Show our image inside it.
      waitKey(1500);
      ++idx; 
    }
    INFO("done");
    l.dump_info();
    cv::destroyAllWindows();
    counterFolder++;
  }
}

// from file img_list.yaml read list [fname]
// [fname1, fname2, ...]
void show_bounding_boxes() 
{
  auto fnames = YAML::LoadFile("img_list.yaml");
  int idx = 0;
  HOGDetectorLogger l("output", &idx);
  HOGDetector d(l);
  for (auto const &fname : fnames) {
    INFO("fname ", fname);
    auto img = cv::imread(fname.as<std::string>());
    auto boxes = d.detect_boxes(img, 0.5);
    ++idx;
  }
  INFO("end");
}