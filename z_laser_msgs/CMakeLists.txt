cmake_minimum_required(VERSION 2.8.3)
project(z_laser_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
)

add_message_files(
   FILES
   Figure.msg
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

catkin_package(
)