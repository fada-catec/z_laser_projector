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


# Unit tests - Level 1 ---------------------
# if (CATKIN_ENABLE_TESTING)
#   catkin_add_nosetests(test/test1.py)
# endif()

# You can also let nosetest find all tests recursively:
# if (CATKIN_ENABLE_TESTING)
#   catkin_add_nosetests(test)
# endif()
# If you used a message of the current package in the nosetest, make sure to specify the nosetest like this:
# catkin_add_nosetests(test/test1.py
#                      DEPENDENCIES ${${PROJECT_NAME}_EXPORTED_TARGETS})
# If you used messages from other packages, use:
catkin_add_nosetests(test/test_manager_connection.py
                     DEPENDENCIES ${catkin_EXPORTED_TARGETS})


# Commands
# cd src/z_laser_projector/z_laser_zlp1/test
# python3 -m unittest test1
# python3 -b -m unittest src/z_laser_projector/z_laser_zlp1/test/test_projector_manager.py

# rosunit -t src/z_laser_projector/z_laser_zlp1/test/test_projector_manager.py

# rosrun z_laser_zlp1 test_projector_manager.py



# Rostests - Level 2 ----------------------------
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/connection_test.test)
  # add_rostest(tests/your_rostest.test ARGS arg1:=true arg2:=false)
endif()


# Commands
# catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_VERSION=3.6
# catkin_make run_tests -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_VERSION=3.6
# catkin_make run_tests_z_laser_zlp1 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_VERSION=3.6
# catkin_test_results build/test_results
# rostest z_laser_zlp1 test2.test
# rostest --text z_laser_zlp1 test2.test
