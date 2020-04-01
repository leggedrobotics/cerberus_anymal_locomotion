cmake_minimum_required(VERSION 2.8.3)
project(cerberus_anymal_control)

set(PACKAGE_DEPENDENCIES
  catkin_simple
  geometry_msgs
  glog_catkin
  roscpp
  roslib
  rospy
  tf
)

find_package(catkin REQUIRED COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

find_package(TensorFlow CONFIG REQUIRED)
if (TensorFlow_FOUND)
 message(STATUS "${PROJECT_NAME}: Found TensorFlow!")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include)

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  INCLUDE_DIRS
    include
)

add_executable(CerberusAnymalControl src/CerberusAnymalControl.cpp)
target_link_libraries(CerberusAnymalControl ${catkin_LIBRARIES} TensorFlow::TensorFlow
${PROJECT_SOURCE_DIR}/src/lib/libcerberus_controller.so)
