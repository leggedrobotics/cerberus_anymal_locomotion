cmake_minimum_required(VERSION 3.10)
project(cerberus_anymal_b_control_1)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Package dependencies
set(PACKAGE_DEPENDENCIES
  cerberus_anymal_control_base
  cerberus_anymal_control_ros
  cerberus_anymal_utils
  eigen_conversions
  geometry_msgs
  roscpp
  roslib
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

## Tensorflow
find_package(TensorFlow CONFIG REQUIRED)
if (TensorFlow_FOUND)
  message(STATUS "${PROJECT_NAME}: Found TensorFlow!")
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/CerberusAnymalBControl1.cpp
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  TensorFlow::TensorFlow
  ${PROJECT_SOURCE_DIR}/src/lib/libcerberus_controller.so
)

## Declare cpp executables
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}.cpp
)
target_compile_features(${PROJECT_NAME}_node INTERFACE cxx_std_11)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
