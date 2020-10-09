cmake_minimum_required(VERSION 2.8.3)
project(z_laser_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  # message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  z_laser_msgs
)

catkin_python_setup()

## CATKIN configuration
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
