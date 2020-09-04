cmake_minimum_required(VERSION 2.8.3)
project(z_laser_projector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  rostest
)

catkin_python_setup()


add_message_files(
   FILES
   Line.msg
)

add_service_files(
   FILES
   ProjectionElement.srv
   CoordinateSystem.srv
   CoordinateSystemName.srv
   CoordinateSystemShow.srv
   CoordinateSystemList.srv
   )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)



## CATKIN configuration
catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)