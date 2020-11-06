#!/usr/bin/env python3

import unittest
import rospy
import rostest

import psutil
from std_srvs.srv import Trigger

from z_laser_zlp1.zlp_projector_manager import ZLPProjectorManager


def remote_ip_present(ip):
    remote_ips = [] # list of IPs for current active connections
    print("ERROR 2 <-------------------------------") # <------------------------------------------
    connections = psutil.net_connections(kind='inet')
    for connection in connections:
        if connection.laddr.ip not in remote_ips:
            remote_ips.append(connection.laddr.ip)

    return ip in remote_ips

class Test2(unittest.TestCase):
    # def setUp(self):
        # check projector is on network
        # if not remote_ip_present("192.168.10.10") or not remote_ip_present("192.168.10.11"):
        #     print("ERROR 3 <-------------------------------") # <------------------------------------------
        #     self.skipTest("projector is not found on network")

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
            
if __name__ == '__main__':
    # rostest.rosrun("z_laser_zlp1", 'test2', Test2, sysargs=['--cov'])
    rostest.rosrun("z_laser_zlp1", 'test2', Test2)
    # rostest.rosrun(package_name, test_name, test_case_class, sysargs='--cov')
    # coverage_packages=['module1.foo', 'module2.bar'] List of packages that should be included in coverage report.


    # Si no funciona sysargs=--cov, se puede hacer el coverage manualmente asi (un ejemplo):
    # from coverage import coverage
    # cov = coverage(
    #     include="*/pr2_pbd_interaction/src/*.py",  # source files
    #     omit="*/src/pr2_pbd_interaction/*"  # generated files
    # )
    # cov.start()

    # Run the system
    # import interaction
    # interaction_ = interaction.Interaction()
    # rospy.spin()

    # System execution finished; generate coverage report if enabled.
    # if use_coverage:
    #     cov.stop()
    #     cov.save()
    #     cov.html_report(title='PR2 PbD code coverage')