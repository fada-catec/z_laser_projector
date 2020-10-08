#!/usr/bin/env python3

import unittest
import rospy
import psutil
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
from z_laser_zlp1.srv import CoordinateSystem, CoordinateSystemRequest, CoordinateSystemResponse

def remote_ip_present(ip):
    remote_ips = [] # list of IPs for current active connections

    connections = psutil.net_connections(kind='inet')
    for connection in connections:
        if connection.laddr.ip not in remote_ips:
            remote_ips.append(connection.laddr.ip)

    return ip in remote_ips

class TestServiceCall(unittest.TestCase):

    # test service should return error
    def test_fail(self):
        # at this point we asume projector is connected
        rospy.wait_for_service("define_coordinate_system")
        s = rospy.ServiceProxy('define_coordinate_system', CoordinateSystem)

        # first try: empty name
        req = CoordinateSystemRequest()
        req.name = ""
        req.distance = 1500     
        req.resolution = 1000
        req.P1 = Point(  0,   0, 0) 
        req.P2 = Point(594,   0, 0) 
        req.P3 = Point(594, 420, 0) 
        req.P4 = Point(  0, 420, 0) 
        req.T1 = Point(  0,   0, 0)

        try:
            resp = s(req)
            if resp.cs_created:
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
            if resp.cs_created:
                self.fail("call failed: we expect the server to throw an exception")
        except Exception as e:
                rospy.loginfo("success. service exception: %s" %e)   

    # test service answer correctly
    def test_no_fail(self):
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
            if not resp.cs_created:
                self.fail("service should have responded successfully")
        except Exception as e:
                self.fail("service should have responded successfully")


if __name__ == '__main__':
    import rostest
    rostest.rosrun("z_laser_zlp1", 'define_coordinate_system_test', TestServiceCall)