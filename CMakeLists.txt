cmake_minimum_required(VERSION 2.8)
project(multi_hypothesis_tracking)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DAPC_CAPTURE_DEBUG=1")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  multi_hypothesis_tracking_msgs
  message_filters
  tf
  tf_conversions
  message_generation
  genmsg
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  )

add_definitions(-std=gnu++11 -fext-numeric-literals)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Libraries
add_library(hypotheses
  src/hypothesis.cpp
  src/kalman_filter.cpp
  )

target_link_libraries(hypotheses
  ${EIGEN3_LIBRARIES}
  )

add_library(hungarian
  src/hungarian.cpp
  )

add_library(multi_hypothesis_tracker
  src/multi_hypothesis_tracker.cpp
  )

target_link_libraries(multi_hypothesis_tracker
  hypotheses
  hungarian
  )

# Nodes
add_executable(multi_hypothesis_tracking_node
  src/multi_hypothesis_tracking_node.cpp
  src/mot_publisher.cpp
  )

target_link_libraries(multi_hypothesis_tracking_node
  multi_hypothesis_tracker
  ${catkin_LIBRARIES}
  )

## Add cmake target dependencies of the executable
add_dependencies(multi_hypothesis_tracking_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# Dummy target for IDE's
FILE(GLOB_RECURSE all_headers_for_ides
  ${CMAKE_SOURCE_DIR}/**/*.hpp
  ${CMAKE_SOURCE_DIR}/**/*.h)
ADD_CUSTOM_TARGET(ide_includes_${PROJECT_NAME} SOURCES ${all_headers_for_ides})