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

catkin_add_nosetests(test/test_viz.py)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/define_coordinate_system_test.test)
  add_rostest(test/projection_element_test.test)
endif()