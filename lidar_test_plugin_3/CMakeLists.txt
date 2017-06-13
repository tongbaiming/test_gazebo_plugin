cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
add_definitions(-std=c++11)
# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(lidar_test_plugin_3 SHARED lidar_test_plugin_3.cc)
target_link_libraries(lidar_test_plugin_2 ${GAZEBO_libraries})
