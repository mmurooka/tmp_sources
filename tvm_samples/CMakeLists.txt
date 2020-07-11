cmake_minimum_required(VERSION 3.1)

project(tvm_samples)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  eigen_conversions
  moveit_msgs
  jsk_rviz_plugins
  )

find_package(TVM REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  roslib
  eigen_conversions
  moveit_msgs
  jsk_rviz_plugins
  DEPENDS
  )

include_directories(${catkin_INCLUDE_DIRS})

add_executable(TvmIkSample
  src/TvmIkSample.cpp
  src/PostureGenerator.cpp)
target_link_libraries(TvmIkSample
  TVM::TVM ${catkin_LIBRARIES})
