#!/usr/bin/env python3

import unittest
import rospy
import socket
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
from z_laser_zlp1.srv import CoordinateSystem, CoordinateSystemRequest, CoordinateSystemResponse

def ip_open(ip,port):
   s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
   try:
      s.connect((ip, int(port)))
      s.shutdown(2)
      return True
   except:
      return False

class TestServiceCall(unittest.TestCase):

    def setUp(self):
        # check projector is on network
        if not ip_open("192.168.10.11",9090):
            self.skipTest("projector is not found on network")

    # test service should return error
    def test1_fail(self):
        # at this point we asume projector is connected
        rospy.wait_for_service("define_coordinate_system")
        s = rospy.ServiceProxy('define_coordinate_system', CoordinateSystem)

        # first try: empty name
        req = CoordinateSystemRequest()
        req.name = ""
        req.distance = 1500     
        req.resolution = 1000
        # TODO es P0,P1 ...
        req.P1 = Point(  0,   0, 0) 
        req.P2 = Point(594,   0, 0) 
        req.P3 = Point(594, 420, 0) 
        req.P4 = Point(  0, 420, 0) 
        req.T1 = Point(  0,   0, 0)

        try:
            resp = s(req)
            if resp.success:
                self.fail("call failed: we expect the server to throw an exception")
        except Exception as e:
                rospy.loginfo("success. service exception: %s" %e)   

        # second try: invalid transformation
        req.name = "test_coordinate_system_invalid"
        req.P1 = Point(100, 250, 0) 
        req.P2 = Point(200,   0, 0) 
        req.P3 = Point(200, 200, 0) 
        req.P4 = Point(  0, 200, 0) 

        try:
            resp = s(req)
            if resp.success:
                self.fail("call failed: we expect the server to throw an exception")
        except Exception as e:
                rospy.loginfo("success. service exception: %s" %e)   

    # test service answer correctly
    def test2_no_fail(self):
        # at this point we asume projector is connected
        rospy.wait_for_service("define_coordinate_system")
        s = rospy.ServiceProxy('define_coordinate_system', CoordinateSystem)

        req = CoordinateSystemRequest()
        req.name = "test_coordinate_system_valid"
        req.distance = 1500
        req.resolution = 1000
        req.P1 = Point(  0,   0, 0) 
        req.P2 = Point(594,   0, 0) 
        req.P3 = Point(594, 420, 0) 
        req.P4 = Point(  0, 420, 0) 
        req.T1 = Point(  0,   0, 0)

        try:
            resp = s(req)
            if not resp.success:
                self.fail("service should have responded successfully")
        except Exception as e:
                self.fail("service should have responded successfully")


if __name__ == '__main__':
    import rostest
    rostest.rosrun("z_laser_zlp1", 'define_coordinate_system_test', TestServiceCall)