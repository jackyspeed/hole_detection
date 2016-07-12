cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

project(hole_detection)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (hole_detection hole_detection.cpp)

target_link_libraries (hole_detection ${PCL_LIBRARIES})
