cmake_minimum_required(VERSION 2.8)
# SET(CMAKE_C_COMPILER             "clang")
# SET(CMAKE_CXX_COMPILER             "clang++")

PROJECT(binloc)

SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
SET(CMAKE_CXX_FLAGS                "-Wall -std=c++11")
SET(CMAKE_CXX_FLAGS_DEBUG          "-g ")
SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
SET(CMAKE_CXX_FLAGS_RELEASE        "-O4 -DNDEBUG")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")

#find_package(OpenCV COMPONENTS core imgproc features2d ml objdetect REQUIRED)
#add_subdirectory (CannyLines)

#find_package( OpenCV REQUIRED )


add_library(${PROJECT_NAME}
    src/binloc.cc
)

target_include_directories (${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/CannyLines/src)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  CannyLines
)


#add_executable(${PROJECT_NAME}_demo
#    src/main.cc
#    src/binloc.cc
#)

#target_include_directories (${PROJECT_NAME}_demo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/CannyLines/src)

#target_link_libraries(${PROJECT_NAME}_demo
#  ${OpenCV_LIBS}
#  CannyLines
#)