cmake_minimum_required(VERSION 2.8)
PROJECT(camera)

SET(CMAKE_C_COMPILER             "/usr/bin/clang")
SET(CMAKE_CXX_COMPILER             "/usr/bin/clang++")
SET(CMAKE_CXX_FLAGS                "-Wall -std=c++11 -pthread")
SET(CMAKE_CXX_FLAGS_DEBUG          "-g ")
SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
SET(CMAKE_CXX_FLAGS_RELEASE        "-O4 -DNDEBUG")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -std=c++11")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")

list (APPEND CMAKE_MODULE_PATH "$ENV{CAMERA_SUITE_PACKAGE}")
find_package(CSPackage REQUIRED)

include_directories($ENV{CAMERA_SUITE_INCLUDE_PATH}/ src/_Common src)

add_library(camera# STATIC
	src/gige_camera.cc
	src/camera.cc
	src/_Common/Common.cpp
	$ENV{CAMERA_SUITE_INCLUDE_PATH}/smcs_cpp/CameraSDK.cpp  
)
 
#add_executable(cam_exec src/main.cc)

target_link_libraries(camera ${cs_LibPath}/${cs_LibName})

#find_package(OpenCV COMPONENTS core imgproc highgui REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#if(OpenCV_FOUND)
	#target_link_libraries(camera ${OpenCV_LIBS})
#endif()
#target_link_libraries(camera ${OpenCV_LIBS})

#target_link_libraries(cam_exec camera)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
