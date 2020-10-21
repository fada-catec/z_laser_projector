cmake_minimum_required(VERSION 2.8.3)
project(z_laser_zlp1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rostest
  z_laser_msgs
)

catkin_python_setup()

## CATKIN configuration
catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)