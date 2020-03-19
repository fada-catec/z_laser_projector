cmake_minimum_required(VERSION 2.8.3)
project(zlaser_sdk_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_python_setup()

add_service_files(
   FILES
   ShapeParams.srv
   CsRefPoints.srv
   ShowCs.srv
   RemovCs.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)



## CATKIN configuration
catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

