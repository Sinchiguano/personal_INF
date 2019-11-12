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



##edited by cesar
list (APPEND CMAKE_MODULE_PATH "$ENV{CAMERA_SUITE_PACKAGE}")
FIND_PACKAGE(CameraSuite REQUIRED)
INCLUDE_DIRECTORIES(${CAMERASUITE_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES(${CAMERASUITE_EXAMPLE_DIR}/cpp/_Common)


#old setup
##look for this packages and then installs it
#find_package(CSPackage REQUIRED)

include_directories(include
                    #/src/_Common
                    /src)


add_library(camera# STATIC
	src/gige_camera.cc
	src/camera.cc
	#src/_Common/Common.cpp
	#${CAMERASUITE_EXAMPLE_DIR}/cpp/_Common

	##old setup
	#$ENV{CAMERA_SUITE_INCLUDE_PATH}/smcs_cpp/CameraSDK.cpp

	##cesar setup
	${CAMERASUITE_INCLUDE_DIRS}/smcs_cpp/CameraSDK.cpp

)

#add_executable(cam_exec src/main.cc)


##from original setup
#target_link_libraries(camera ${cs_LibPath}/${cs_LibName})
target_link_libraries(camera ${CAMERASUITE_LIBRARY})


##cesar setup
#TARGET_LINK_LIBRARIES(ContinuousGrabUserBuffer_cpp  ${CAMERASUITE_LIBRARY})


set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
