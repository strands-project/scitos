cmake_minimum_required(VERSION 2.8.3)
project(openni_wrapper)

set(CMAKE_CXX_FLAGS "-g -fPIC -std=c++0x -fpermissive")

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge)
find_package( OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)

set(OPENNI2_DIR /home/rares/code/OpenNI-Linux-x64-2.2/)
link_directories(${OPENNI2_DIR}/Redist)
include_directories(${OPENNI2_DIR}/Include)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

set(HDRS
    include/ColorCallback.h
    include/DepthCallback.h
    include/OniSampleUtilities.h
)

set(SRCS
    src/ColorCallback.cpp
    src/DepthCallback.cpp
    src/main.cpp
)

add_executable(openni_wrapper ${HDRS} ${SRCS})

 target_link_libraries(openni_wrapper
   ${catkin_LIBRARIES} OpenNI2 ${OpenCV_LIBS}
 )
