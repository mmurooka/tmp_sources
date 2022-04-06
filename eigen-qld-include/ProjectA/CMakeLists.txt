cmake_minimum_required(VERSION 3.1)
project(ProjectA)

find_package(eigen-qld REQUIRED)

add_library(ProjectA SHARED src/ProjectA.cpp)
target_include_directories(ProjectA PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ProjectA PRIVATE eigen-qld::eigen-qld)

install(TARGETS ProjectA
  EXPORT ProjectAConfig
  LIBRARY DESTINATION lib)
install(EXPORT ProjectAConfig
  NAMESPACE ProjectA::
  DESTINATION lib/cmake/ProjectA)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include)
