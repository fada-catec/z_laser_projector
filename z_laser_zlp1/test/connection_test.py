#!/usr/bin/env python3

import unittest
import rospy
import psutil
from std_srvs.srv import Trigger

import socket

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

    # the node is automatically connected when launched, so perform disconnection first

    # test service should return error
    def test1_disconnection_fail(self):
        rospy.wait_for_service("disconnect")
        s = rospy.ServiceProxy('disconnect', Trigger)
        try:
            resp = s("data") # call std_srvs/Trigger with values should fails
            if resp.success:
                self.fail("call failed: we expect the server to throw an exception")
        except TypeError as e:
                rospy.loginfo("success. service exception: %s" %e)   

    # test service answer correctly
    def test2_disconnection_no_fail(self):
        rospy.wait_for_service("disconnect")
        s = rospy.ServiceProxy('disconnect', Trigger)

        # if projector is reachable, service should disconnect
        resp = s()
        if resp.success:
            pass
        else:
            self.fail("service should have responded successfully")

    # once disconnected, reconnect again

    # test service should return error
    def test3_connection_fail(self):
        rospy.wait_for_service("connect")
        s = rospy.ServiceProxy('connect', Trigger)
        try:
            resp = s("data") # call std_srvs/Trigger with values should fails
            if resp.success:
                self.fail("call failed: we expect the server to throw an exception")
        except TypeError as e:
                rospy.loginfo("success. service exception: %s" %e)   

    # test service answer correctly
    def test4_connection_no_fail(self):
        rospy.wait_for_service("connect")
        s = rospy.ServiceProxy('connect', Trigger)

        # if projector is reachable, service should connect
        resp = s()
        if resp.success:
            pass
        else:
            self.fail("service should have responded successfully")
            
if __name__ == '__main__':
    import rostest
    rostest.rosrun("z_laser_zlp1", 'connection_test', TestServiceCall)