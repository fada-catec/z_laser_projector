#!/usr/bin/env python3

import sys
import os
import unittest
import rosunit
import rospy
import socket

from z_laser_zlp1.zlp_projector_manager import ZLPProjectorManager

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

class TestProjectorManager(unittest.TestCase):

    def setUp(self):
        # check projector is on network
        if not ip_open("192.168.10.11",9090):
            self.skipTest("projector is not found on network")

    # test connection with empty license path should return error
    def test1_no_license_path(self):
        
        projector_manager = ZLPProjectorManager(projector_IP = "192.168.10.10", 
                                                server_IP = "192.168.10.11", 
                                                connection_port = 9090, 
                                                lic_path="")

        try:
            projector_manager.connect_and_setup()
            self.fail("error test1")

        except Exception:
            pass

    # test connection with wrong server IP should return error
    def test2_no_server_IP(self):

        projector_manager = ZLPProjectorManager(projector_IP = "192.168.10.10", 
                                                server_IP = "", 
                                                connection_port = 9090, 
                                                lic_path="")

        try:
            projector_manager.connect_and_setup()
            self.fail("error test2")

        except Exception:
            pass

    # test disconnection should answer correctly
    def test3_diconnection(self):

        license_path = (os.path.dirname(os.path.dirname(os.path.abspath(__file__)))+'/lic/1900027652.lic')
        projector_manager = ZLPProjectorManager(projector_IP = "192.168.10.10", 
                                                server_IP = "192.168.10.11", 
                                                connection_port = 9090, 
                                                lic_path=license_path)

        try:
            projector_manager.connect_and_setup()
            status = projector_manager.get_connection_status()
            if status:
                projector_manager.deactivate()
                projector_manager.client_server_disconnect()
                pass
            else:
                self.fail("error test3")

        except Exception:
            self.fail("error test3")

    # test start projection without active coordinate system defined should return error
    def test4_start_proj(self):

        license_path = (os.path.dirname(os.path.dirname(os.path.abspath(__file__)))+'/lic/1900027652.lic')
        projector_manager = ZLPProjectorManager(projector_IP = "192.168.10.10", 
                                                server_IP = "192.168.10.11", 
                                                connection_port = 9090, 
                                                lic_path=license_path)

        try:
            projector_manager.connect_and_setup()
            status = projector_manager.get_connection_status()
            if status:
                projector_manager.start_projection()
                projector_manager.deactivate()
                projector_manager.client_server_disconnect()
                pass
            else:
                self.fail("error test4")

        except Exception:
            projector_manager.deactivate()
            projector_manager.client_server_disconnect()
            pass


if __name__ == '__main__':
    print("main")

    rosunit.unitrun('z_laser_zlp1', 'test_projector_manager', TestProjectorManager,  sysargs=None)
    # rosunit.unitrun('z_laser_zlp1', 'test1', Test1,  sysargs=None, coverage_packages=['z_laser_zlp1'])
    # rosunit.unitrun('z_laser_zlp1', 'test1', Test1,  sysargs=['--cov'])
    # rosunit.unitrun(package_name, test_name, test_case_class, sysargs=None, coverage_packages=None)
    # coverage_packages=['module1.foo', 'module2.bar'] List of packages that should be included in coverage report.
