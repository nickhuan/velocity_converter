cmake_minimum_required(VERSION 2.8.3)
project(velocity_converter)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  OpenCV REQUIRED
)

catkin_package(
  # DEPENDS cv_bridge
  INCLUDE_DIRS include
#  LIBRARIES velocity_converter
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  include/velocity_converter

)

add_executable(converter
  src/converter.cpp
  include/velocity_converter/vc_node.h
  src/vc_node.cpp
)
add_dependencies(converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(converter
  ${catkin_LIBRARIES}
)

add_library(vc_node
  include/velocity_converter/vc_node.h
  src/vc_node.cpp
)
add_dependencies(vc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vc_node
  ${catkin_LIBRARIES}
)