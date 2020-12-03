#!/usr/bin/env python3

import sys
import unittest
import rosunit
import rospy
import socket

from z_laser_zlp1.zlp_projector_manager import ZLPProjectorManager

def ip_open(ip,port):
    print("ip_open")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print("connecting")
        s.connect((ip, int(port)))
        print("trying connect")
        s.shutdown(2)
        print("end shutdown")
        return True
    except:
        print("not connect")
        return False

class TestProjectorManager(unittest.TestCase):

    def setUp(self):
        # check projector is on network
        print("setup ip_open")
        if not ip_open("192.168.10.11",9090):
            self.skipTest("projector is not found on network")

    # test connection with empty license path should return error
    def test1_no_license_path(self):

        print("test1")

        projector_manager = ZLPProjectorManager(projector_IP = "192.168.10.10", 
                                                server_IP = "192.168.10.11", 
                                                connection_port = 9090, 
                                                lic_path="")
        try:
            projector_manager.connect_and_setup()
            self.fail("error test1_connection_and_setup")

        except Exception as e:
            pass

    # test connection with wrong server IP should return error
    def test2_no_server_IP(self):

        print("test2")

        projector_manager = ZLPProjectorManager(projector_IP = "192.168.10.10", 
                                                server_IP = "", 
                                                connection_port = 9090, 
                                                lic_path="")
        try:
            projector_manager.connect_and_setup()
            self.fail("error test2_connection_and_setup")

        except Exception as e:
            pass

if __name__ == '__main__':
    print("main")

    rosunit.unitrun('z_laser_zlp1', 'test_projector_manager', TestProjectorManager,  sysargs=None)
