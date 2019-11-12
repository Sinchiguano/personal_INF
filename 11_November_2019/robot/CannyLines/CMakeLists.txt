PROJECT( CannyLines )

#FIND_PACKAGE(OpenCV REQUIRED)
#find_package(OpenCV COMPONENTS core imgproc features2d ml objdetect REQUIRED)
#FIND_PACKAGE(OpenCV COMPONENTS core highgui REQUIRED)

FILE(GLOB_RECURSE HDRS_FILES *.h *.hpp)
FILE(GLOB_RECURSE SRCS_FILES *.c *.cpp)

ADD_LIBRARY(${PROJECT_NAME} 
src/CannyLine.cpp
src/MetaLine.cpp
src/buffer_2d.cpp
)

message("OPENCVLIBS " ${OpenCV_LIBS})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})