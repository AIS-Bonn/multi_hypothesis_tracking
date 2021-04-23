cmake_minimum_required(VERSION 2.8)
project(multi_hypothesis_tracking)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DAPC_CAPTURE_DEBUG=1")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  person_msgs
  multi_hypothesis_tracking_msgs
  pcl_ros
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  visualization_msgs
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

# Library for a hypothesis using a Kalman Filter for pose updates
add_library(hypothesis
  src/hypothesis.cpp
  src/kalman_filter.cpp
)
target_link_libraries(hypothesis
  ${EIGEN3_LIBRARIES}
)

# Library for the hungarian method used for assigning detections to hypotheses
add_library(hungarian
  src/hungarian.cpp
)

# Library for the Multi Hypothesis Tracker combining hypotheses and assignment method
add_library(multi_hypothesis_tracker
  src/multi_hypothesis_tracker.cpp
)
target_link_libraries(multi_hypothesis_tracker
  hypothesis
  hungarian
)

# Library for the Multi Hypothesis Tracker combining hypotheses and assignment method
add_library(visualizations_publisher
  src/visualizer/visualizations_publisher.cpp
)
target_link_libraries(visualizations_publisher
  hypothesis
  ${catkin_LIBRARIES}
)
add_dependencies(visualizations_publisher
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Library for Multi Hypothesis Tracking Base class - nodes inherit and implement callback methods
add_library(${PROJECT_NAME}_base
  src/${PROJECT_NAME}_base.cpp
)
target_link_libraries(${PROJECT_NAME}_base
  multi_hypothesis_tracker
  visualizations_publisher
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_base
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Ros node as interface between incoming messages and tracker
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_base
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Ros node as interface between incoming human_msgs and tracker
add_executable(${PROJECT_NAME}_node_for_humans
  src/multi_hypothesis_tracking_node_for_humans.cpp
)
target_link_libraries(${PROJECT_NAME}_node_for_humans
  ${PROJECT_NAME}_base
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node_for_humans
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Dummy target for IDE's
FILE(GLOB_RECURSE all_headers_for_ides
  ${CMAKE_SOURCE_DIR}/**/*.hpp
  ${CMAKE_SOURCE_DIR}/**/*.h)
ADD_CUSTOM_TARGET(ide_includes_${PROJECT_NAME} SOURCES ${all_headers_for_ides})