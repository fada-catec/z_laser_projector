cmake_minimum_required(VERSION 2.8.3)
project(z_laser_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
)

catkin_python_setup()


add_message_files(
   FILES
   Line.msg
   Curve.msg
   Text.msg
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
   geometry_msgs
)


## CATKIN configuration
catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)