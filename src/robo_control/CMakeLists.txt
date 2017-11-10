cmake_minimum_required(VERSION 2.8.3)
project(robo_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(robo_control robo_control.cpp)

target_link_libraries(robo_control ${catkin_LIBRARIES})
