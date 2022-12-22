cmake_minimum_required(VERSION 3.1)

project(ocs2_ipm_sample LANGUAGES CXX)
find_package(catkin REQUIRED COMPONENTS
  ocs2_ipm
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  CATKIN_DEPENDS
  ocs2_ipm
  DEPENDS EIGEN3
)

add_executable(SampleOscillator src/SampleOscillator.cpp)
include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(SampleOscillator PRIVATE ${catkin_LIBRARIES})
