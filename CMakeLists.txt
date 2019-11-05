cmake_minimum_required(VERSION 2.8.3)
project(yolo_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
cv_bridge#
image_transport#
std_msgs
genmsg
sensor_msgs
message_generation

)
#in order to work with the openCV in my project
find_package(OpenCV REQUIRED)



## Declare ROS messages and services
add_message_files(
  FILES
    BoundingBox.msg
    BoundingBoxes.msg
    ObjectCount.msg
)
#add_service_files(FILES AddTwoInts.srv)
add_service_files(
   FILES
   AddTwoInts.srv
   CheckForObjects.srv
   YoloDetection.srv
 )

## Generate added messages and services
generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs)


## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    sensor_msgs
    cv_bridge#
    image_transport# 
    roscpp #
    sensor_msgs# 
    std_msgs#
)

#catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp message_runtime sensor_msgs
#)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(yoloCLIENT src/yoloCLIENT.cc)
target_link_libraries(yoloCLIENT ${catkin_LIBRARIES}  ${OpenCV_LIBS})
add_dependencies(yoloCLIENT yolo_detector_messages_cpp)

#add_executable(YoloClientTmp src/YoloClientTmp.cc)
#target_link_libraries(YoloClientTmp ${catkin_LIBRARIES}  ${OpenCV_LIBS})
#add_dependencies(YoloClientTmp yolo_detector_messages_cpp)


#add_executable(yoloRequest src/yoloRequest.cpp)
#target_link_libraries(yoloRequest ${catkin_LIBRARIES}  ${OpenCV_LIBS})
#add_dependencies(yoloRequest yolo_detector_messages_cpp)


#add_executable(yoloDetection src/yoloDetection.cpp)
#target_link_libraries(yoloDetection ${catkin_LIBRARIES}  ${OpenCV_LIBS})
#add_dependencies(yoloDetection yolo_detector_messages_cpp)

#add_executable(YoloClient src/YoloClient.cc)
#target_link_libraries(YoloClient ${catkin_LIBRARIES}  ${OpenCV_LIBS})
#add_dependencies(YoloClient yolo_detector_messages_cpp)

#add_executable(talker_ src/talker_.cpp)
#target_link_libraries(talker_ ${catkin_LIBRARIES})
#add_dependencies(talker_ yolo_detector_messages_cpp)

#add_executable(listener_ src/listener_.cpp)
#target_link_libraries(listener_ ${catkin_LIBRARIES})
#add_dependencies(listener_ yolo_detector_messages_cpp)
