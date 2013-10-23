cmake_minimum_required(VERSION 2.8.3)
project(openni_wrapper)

set(CMAKE_CXX_FLAGS "-g -fPIC -std=c++0x -fpermissive")

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge nodelet class_loader)
find_package( OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)

set(OPENNI2.2_DIR ../OpenNI2/)
link_directories(${OPENNI2.2_DIR}/Bin/x64-Release)
include_directories(${OPENNI2.2_DIR}/Include)


execute_process(COMMAND make WORKING_DIRECTORY ${OPENNI2.2_DIR})


catkin_package(
   INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

set(HDRS
    include/ColorCallback.h
    include/OpenniWrapperNodelet.h
    include/DepthCallback.h
    include/OniSampleUtilities.h
    include/OpenniWrapperNode.h
)

set(SRCS
    src/ColorCallback.cpp
    src/OpenniWrapperNodelet.cpp
    src/DepthCallback.cpp
    src/OpenniWrapperNode.cpp
    src/main.cpp
)

add_executable(openni_wrapper ${HDRS} ${SRCS})

add_library(openni_wrapper_lib  ${HDRS} ${SRCS})

 target_link_libraries(openni_wrapper
   ${catkin_LIBRARIES} OpenNI2 ${OpenCV_LIBS}
 )

 target_link_libraries(openni_wrapper_lib
   ${catkin_LIBRARIES} OpenNI2 ${OpenCV_LIBS}
 )
