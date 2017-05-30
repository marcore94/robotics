cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#Plugin used to control forks through a PID controller
add_library(ControlPlugin SHARED ControlPlugin.cc)
target_link_libraries(ControlPlugin ${GAZEBO_libraries})
