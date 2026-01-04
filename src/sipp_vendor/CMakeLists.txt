cmake_minimum_required(VERSION 3.0.2)
project(sipp_vendor)

find_package(catkin REQUIRED)

catkin_package(
  LIBRARIES sipp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/SIPP
  ${CMAKE_CURRENT_SOURCE_DIR}/SIPP/SIPP
)

file(GLOB_RECURSE SIPP_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/SIPP/*.cpp
)

add_library(sipp ${SIPP_SRC})
