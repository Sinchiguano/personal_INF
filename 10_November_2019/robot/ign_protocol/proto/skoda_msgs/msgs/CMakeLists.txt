find_package(Protobuf REQUIRED)

include_directories(
  ${PROTOBUF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

set(PROTOBUF_IMPORT_DIRS "${IGNITION-MSGS_INCLUDE_DIRS}")
message(STATUS "INCDIR: ${IGNITION-MSGS_INCLUDE_DIRS}")

set(msgs
  errormsg.proto
  statusmsg.proto
  commandmsg.proto
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})

add_library(skoda_msgs SHARED ${PROTO_SRCS})
target_link_libraries(skoda_msgs ${PROTOBUF_LIBRARY})
