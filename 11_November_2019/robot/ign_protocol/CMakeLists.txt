cmake_minimum_required(VERSION 2.8)
project(ign_protocol)

add_definitions('-std=c++11')

find_package(ignition-msgs1 QUIET REQUIRED)
find_package(ignition-transport4 QUIET REQUIRED)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")

# Runs moc automatically when required.
# TODO: have it work with the current directory structure.
#set(CMAKE_AUTOMOC TRUE)
# Find includes in corresponding build directories.
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

include_directories(
  include
  ${CMAKE_BINARY_DIR}
  ${CMAKE_BINARY_DIR}/robot/ign_protocol/proto
  ${IGNITION-MSGS_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
)


add_library(fake_core
  src/fake_core.cpp
  ${PROTO_SRCS}
)

target_link_libraries(fake_core
  skoda_msgs
  ${IGNITION-TRANSPORT_LIBRARIES}
)

add_subdirectory(proto/skoda_msgs/msgs)

