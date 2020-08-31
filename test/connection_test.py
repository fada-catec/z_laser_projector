#!/usr/bin/env python3

import unittest
import rospy
import psutil
from std_srvs.srv import Trigger

def remote_ip_present(ip):
    remote_ips = [] # list of IPs for current active connections

    connections = psutil.net_connections(kind='inet')
    for connection in connections:
        if connection.laddr.ip not in remote_ips:
            remote_ips.append(connection.laddr.ip)

    return ip in remote_ips

class TestServiceCall(unittest.TestCase):
    def setUp(self):
        # check projector is on network
        if not remote_ip_present("192.168.10.10") or not remote_ip_present("192.168.10.11"):
            self.skipTest("projector is not found on network")

    # the node is automatically connected when launched, so perform disconnection first

    # test service should return error
    def test_disconnection_fail(self):
        rospy.wait_for_service("disconnect")
        s = rospy.ServiceProxy('disconnect', Trigger)
        try:
            resp = s("data") # call std_srvs/Trigger with values should fails
            if resp.success:
                self.fail("call failed: we expect the server to throw an exception")
        except TypeError as e:
                rospy.loginfo("success. service exception: %s" %e)   

    # test service answer correctly
    def test_disconnection_no_fail(self):
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
    def test_connection_fail(self):
        rospy.wait_for_service("connect")
        s = rospy.ServiceProxy('connect', Trigger)
        try:
            resp = s("data") # call std_srvs/Trigger with values should fails
            if resp.success:
                self.fail("call failed: we expect the server to throw an exception")
        except TypeError as e:
                rospy.loginfo("success. service exception: %s" %e)   

    # test service answer correctly
    def test_connection_no_fail(self):
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
    rostest.rosrun("z_laser_projector", 'connection_test', TestServiceCall)