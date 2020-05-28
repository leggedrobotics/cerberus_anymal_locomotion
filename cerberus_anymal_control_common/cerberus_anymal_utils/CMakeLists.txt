cmake_minimum_required(VERSION 3.10)
project(cerberus_anymal_utils)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS
    include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)