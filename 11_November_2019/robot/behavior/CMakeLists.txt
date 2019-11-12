project( behavior )
cmake_minimum_required( VERSION 2.6 )

aux_source_directory( src SRC )
include_directories( src )

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
set(ZeroMQ_DIR "${CMAKE_SOURCE_DIR}/robot/cmake/Modules/")

#check_cxx_compiler_flag( "-std=c++14"   COMPILER_SUPPORTS_CXX14 )
set(CMAKE_CXX_FLAGS -g)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# generate_proto(command_msg proto/skoda_msgs/msgs/commandmsg.proto)

include_directories(
  ${CMAKE_BINARY_DIR}
  ${CMAKE_BINARY_DIR}/robot/ign_protocol/proto
  ${IGNITION-MSGS_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
)

find_package( Boost COMPONENTS system filesystem REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )
link_directories( ${Boost_LIBRARY_DIRS} )
set( Boost_USE_STATIC_LIBS        OFF )
set( Boost_USE_MULTITHREADED      ON )
set( Boost_USE_STATIC_RUNTIME     OFF )
set( BOOST_ALL_DYN_LINK           ON )

find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )

find_package (ZeroMQ REQUIRED)
include_directories( ${ZMQ_INCLUDE_DIR} )


########cesar###########################
#find_package (yolo_detector REQUIRED)


#add_executable(${PROJECT_NAME} ${SRC})
add_library(${PROJECT_NAME} ${SRC})

#link_directories("/home/binpicking/catkin_ws/third_party/ignition_protocol/build")
#target_link_libraries( ${PROJECT_NAME} /home/binpicking/catkin_ws/third_party/ignition_protocol/build/libcore.so)
#include_directories("/home/binpicking/catkin_ws/third_party/ignition_protocol/include")

target_link_libraries( ${PROJECT_NAME} ${Boost_LIBRARIES} )
target_link_libraries( ${PROJECT_NAME} ${ZMQ_LIBRARY} )
target_link_libraries( ${PROJECT_NAME} camera )




find_package(yaml-cpp REQUIRED)


include_directories(${YAML_CPP_INCLUDE_DIR})
target_link_libraries( ${PROJECT_NAME} ${YAML_CPP_LIBRARIES} )

#find_package(OpenCV COMPONENTS core imgproc features2d ml objdetect REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
if(OpenCV_FOUND)
	target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif()



target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} fake_core)
find_library(DAQ_LIB biodaq biodaqutil)

target_link_libraries(${PROJECT_NAME} ${DAQ_LIB})
##target_link_libraries(${PROJECT_NAME} biodaqutil)
 
add_executable(hog src/mains/hog.cpp)
target_link_libraries(hog behavior)
#add_dependencies(hog ${catkin_EXPORTED_TARGETS} yolo_detector_messages_cpp)


#################################################
add_executable(behave src/mains/behavior.cpp)
target_link_libraries(behave behavior )
#add_dependencies(behave yolo_detector_messages_cpp)# added by cesar

add_executable(test_detection src/mains/test_detection.cpp)
target_link_libraries(test_detection behavior )
#add_dependencies(test_detection yolo_detector_messages_cpp)# added by cesar

##target_link_libraries(behave ${IGNITION-TRANSPORT_LIBRARIES} )

catkin_add_gtest(utest test/convert_3d.cpp)
target_link_libraries(utest behavior ) 
#add_dependencies(utest yolo_detector_messages_cpp)# added by cesar


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
