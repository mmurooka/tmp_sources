cmake_minimum_required(VERSION 3.1)
project(ProjectB)

find_package(eigen-qld REQUIRED) # Is it possible to skip this???
find_package(ProjectA REQUIRED)

add_library(ProjectB SHARED src/ProjectB.cpp)
target_include_directories(ProjectB PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ProjectB PUBLIC ProjectA::ProjectA)

install(TARGETS ProjectB)
