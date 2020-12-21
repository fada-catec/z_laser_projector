#!/usr/bin/env python3

import unittest
import rospy
import socket
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
from z_laser_msgs.srv import CoordinateSystem, CoordinateSystemRequest
from z_laser_msgs.srv import CoordinateSystemList, CoordinateSystemShow, CoordinateSystemName

def ip_open(ip,port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2)
    try:
        s.connect((ip, int(port)))
        success = True
    except:
        success = False
    s.close()
    return success

class TestServiceCall(unittest.TestCase):

    def setUp(self):
        # check projector is on network
        if not ip_open("192.168.10.11",9090):
            self.skipTest("projector is not found on network")

    # test service should return error
    def test1_fail_define_cs(self):
        # at this point we asume projector is connected
        rospy.wait_for_service("define_coordinate_system")
        s = rospy.ServiceProxy('define_coordinate_system', CoordinateSystem)

        # first try: empty name
        req = CoordinateSystemRequest()
        req.name = ""
        req.distance = 1500     
        req.resolution = 1000
        req.T0 = Point(  0,   0, 0)
        req.P.append(Point(  0,   0, 0)) 
        req.P.append(Point(594,   0, 0)) 
        req.P.append(Point(594, 420, 0)) 
        req.P.append(Point(  0, 420, 0)) 

        try:
            resp = s(req)
            self.fail("call failed: expect the server to throw an exception")            
        except Exception as e:
            rospy.loginfo("success. service exception: %s" %e)

        # second try: invalid transformation
        req.name = "test_coordinate_system_invalid"
        req.P.append(Point(100, 250, 0)) 
        req.P.append(Point(200,   0, 0)) 
        req.P.append(Point(200, 200, 0)) 
        req.P.append(Point(  0, 200, 0)) 
        
        try:
            resp = s(req)
            self.fail("call failed: expect the server to throw an exception")
        except Exception as e:
            rospy.loginfo("success. service exception: %s" %e)   

    # test service answer correctly
    def test2_no_fail_define_cs(self):
        # at this point we asume projector is connected
        rospy.wait_for_service("define_coordinate_system")
        s = rospy.ServiceProxy('define_coordinate_system', CoordinateSystem)

        req = CoordinateSystemRequest()
        req.name = "test_coordinate_system_valid"
        req.distance = 1500
        req.resolution = 1000
        req.T0 = Point(  0,   0, 0)
        req.P.append(Point(  0,   0, 0))
        req.P.append(Point(594,   0, 0))
        req.P.append(Point(594, 420, 0))
        req.P.append(Point(  0, 420, 0))

        try:
            resp = s(req)
            if not resp.success:
                self.fail("service should have responded successfully")
        except Exception as e:
            self.fail("service should have responded successfully")

    # test service answer correctly
    def test3_get_cs(self):
        # at this point we asume projector is connected
        rospy.wait_for_service("coordinate_system_list")
        s = rospy.ServiceProxy('coordinate_system_list', CoordinateSystemList)

        try:
            resp = s()
            if not resp.success:
                self.fail("service should have responded successfully")
            
            self.assertGreater(len(resp.cs_list), 1)
            self.assertEquals(resp.active_cs, "test_coordinate_system_valid")

        except Exception as e:
            self.fail("service should have responded successfully")

    # test service answer correctly
    def test4_manage_cs(self):
        # at this point we asume projector is connected
        rospy.wait_for_service("show_active_coordinate_system")
        rospy.wait_for_service("remove_coordinate_system")
        show = rospy.ServiceProxy('show_active_coordinate_system', CoordinateSystemShow)
        rem = rospy.ServiceProxy('remove_coordinate_system', CoordinateSystemName)
        
        try:
            resp = show(2)
            if not resp.success:
                self.fail("service should have responded successfully")
            
            resp = rem("test_coordinate_system_valid")
            if not resp.success:
                self.fail("service should have responded successfully")

        except Exception as e:
            self.fail("service should have responded successfully")


if __name__ == '__main__':
    import rostest
    rostest.rosrun("z_laser_zlp1", 'define_coordinate_system_test', TestServiceCall)