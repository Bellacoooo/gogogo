cmake_minimum_required(VERSION 3.0.2)
project(flight_data_recorder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  geometry_msgs
  std_msgs
  onboard_detector
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy nav_msgs geometry_msgs std_msgs onboard_detector
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(data_recorder_node src/data_recorder_node.cpp)
target_link_libraries(data_recorder_node ${catkin_LIBRARIES})
add_dependencies(data_recorder_node ${catkin_EXPORTED_TARGETS})



