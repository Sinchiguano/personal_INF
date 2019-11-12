project(robot)
cmake_minimum_required( VERSION 2.6 )

find_package(ignition-transport4 QUIET REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  cv_bridge#casch
  #image_transport#casch
  sensor_msgs#casch
  rospy
  std_msgs
  message_generation
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
  geometric_shapes
  tf
  sonars
)
#find_package(OpenCV COMPONENTS core highgui REQUIRED)

catkin_package(
CATKIN_DEPENDS 
roscpp 
rospy 
std_msgs 
cv_bridge#casch
sensor_msgs#casch
geometry_msgs 
moveit_core 
moveit_ros_planning_interface
#  DEPENDS
#OpenCV
)
find_package(OpenCV COMPONENTS core imgproc features2d ml objdetect REQUIRED)


#include_directories(${ignition_ROS_interface_INCLUDE_DIRS})
#include_directories("/home/binpicking/catkin_ws/src/ignition_ROS_interface/include/")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(ignition-transport4 QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")

include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})

link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

message("include_directories " ${catkin_INCLUDE_DIRS})

#add_subdirectory(msgs)
#set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES GENERATED TRUE)
#include_directories(${CMAKE_BINARY_DIR})

add_subdirectory(CannyLines)
#add_subdirectory(binlock)
#find_package(OpenCV COMPONENTS core imgproc highgui REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(camera)
include_directories(camera/src)
include_directories($ENV{CAMERA_SUITE_INCLUDE_PATH})

add_subdirectory(behavior)
add_subdirectory(ign_protocol)
# add_subdirectory(grpc_protocol)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
