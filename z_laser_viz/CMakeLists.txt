cmake_minimum_required(VERSION 2.8.3)
project(z_laser_viz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  z_laser_msgs
  z_laser_zlp1
  tf
)

catkin_python_setup()

## CATKIN configuration
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
