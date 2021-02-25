cmake_minimum_required(VERSION 2.8.3)
project(z_laser_zlp1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rostest
  z_laser_msgs
)

catkin_python_setup()

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_utils.py)
  catkin_add_nosetests(test/test_manager_connection.py)
  catkin_add_nosetests(test/test_manager_coordinate_system.py)                     
  catkin_add_nosetests(test/test_manager_projection_element.py)                     
  catkin_add_nosetests(test/test_dxf.py)  

  find_package(rostest REQUIRED)
  add_rostest(test/connection_test.test)
  add_rostest(test/define_coordinate_system_test.test)
  add_rostest(test/projection_element_test.test)
endif()