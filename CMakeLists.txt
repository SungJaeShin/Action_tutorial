cmake_minimum_required(VERSION 3.0.2)
project(Action_tutorial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  image.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs   
  std_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
#  LIBRARIES Action_tutorial
 CATKIN_DEPENDS actionlib roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
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

add_executable(client src/ImgClient.cpp)
target_link_libraries(client ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(server src/ImgServer.cpp)
target_link_libraries(server ${catkin_LIBRARIES} ${OpenCV_LIBS})
