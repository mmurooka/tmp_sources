cmake_minimum_required(VERSION 3.1)

project(mc_rtc_ik LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  eigen_conversions
  optmotiongen_msgs
  optmotiongen_rviz_plugins
)

# mc_rtc
find_package(mc_rtc REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  eigen_conversions
  optmotiongen_msgs
  optmotiongen_rviz_plugins
)

add_executable(sample src/sample.cpp)
include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(sample PRIVATE ${catkin_LIBRARIES} mc_rtc::mc_tasks)
