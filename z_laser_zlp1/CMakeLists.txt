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

catkin_add_nosetests(test/test_manager_connection.py
                     DEPENDENCIES ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/define_coordinate_system_test.test)
endif()