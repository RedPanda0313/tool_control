cmake_minimum_required(VERSION 3.0.2)
project(tool_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  controller_manager
)

set(EXTRALIB_DIR ${PROJECT_SOURCE_DIR}/lib)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tool_control
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EXTRALIB_DIR}/libbiodaq.so
  )
add_executable(tool_control
  src/main.cpp
  src/tool_hw.cpp
  src/tool_servo.cpp
  #src/rotation_openclose.cpp
)
target_link_libraries(tool_control
  ${catkin_LIBRARIES}
  ${EXTRALIB_DIR}/libbiodaq.so
 )

add_executable(tool_adjust
  src/tool_adjust/main.cpp
)
target_link_libraries(tool_adjust
  ${catkin_LIBRARIES}
 )
 add_executable(tool_control_test
  src/tool_test/main.cpp
)
 
target_link_libraries(tool_control_test
  ${catkin_LIBRARIES}
 )