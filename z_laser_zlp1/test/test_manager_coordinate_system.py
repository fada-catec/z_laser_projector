#!/usr/bin/env python3

import sys
import os
import unittest
import rosunit
import rospy
import socket
from copy import deepcopy

from z_laser_zlp1.zlp_projector_manager import ZLPProjectorManager
from z_laser_zlp1.zlp_utils import  CoordinateSystemParameters
from z_laser_msgs.msg import Figure
from z_laser_msgs.srv import ProjectionElementRequest

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
        license_path = (os.path.dirname(os.path.dirname(os.path.abspath(__file__)))+'/lic/1900027652.lic')
        self.projector_manager = ZLPProjectorManager(projector_IP = "192.168.10.10", 
                                                server_IP = "192.168.10.11", 
                                                connection_port = 9090, 
                                                lic_path=license_path)

    def tearDown(self):
        self.projector_manager.deactivate()
        self.projector_manager.client_server_disconnect()

    # test creation of invalid coordinate system
    def test1_create_wrong_coordinate_system(self):
        # coordinate system
        cs_params            = CoordinateSystemParameters()
        cs_params.name       = "coordinate_system_test"
        cs_params.distance   = 1500
        cs_params.P[0].x     = 100
        cs_params.P[0].y     = 250
        cs_params.P[1].x     = 200
        cs_params.P[1].y     =   0
        cs_params.P[2].x     = 200
        cs_params.P[2].y     = 200
        cs_params.P[3].x     =   0
        cs_params.P[3].y     = 200
        cs_params.T[0].x     =   0
        cs_params.T[0].y     =   0
        cs_params.resolution = 1000

        try:
            self.projector_manager.connect_and_setup()
            status = self.projector_manager.get_connection_status()
            if status:
                self.projector_manager.define_coordinate_system(cs_params, False)
                self.fail("Library accepted invalid data")
            else:
                self.fail("connection error")

        except Exception as e:
            pass

    # test creation of valid coordinate system
    def test2_create_coordinate_system(self):
        name1 = "coordinate_system_test_1"
        name2 = "coordinate_system_test_2"
        # coordinate system 1
        cs_params1            = CoordinateSystemParameters()
        cs_params1.name       = name1
        cs_params1.distance   = 1500
        cs_params1.P[0].x     = -100
        cs_params1.P[0].y     = -100
        cs_params1.P[1].x     =  100
        cs_params1.P[1].y     = -100
        cs_params1.P[2].x     =  100
        cs_params1.P[2].y     =  100
        cs_params1.P[3].x     = -100
        cs_params1.P[3].y     =  100
        cs_params1.T[0].x     =    0
        cs_params1.T[0].y     =    0
        cs_params1.resolution = 1000
        # coordinate system 2
        cs_params2 = deepcopy(cs_params1)
        cs_params2.name     = name2
        cs_params2.distance = 2000

        try:
            self.projector_manager.connect_and_setup()
            status = self.projector_manager.get_connection_status()
            if status:
                # create coordinate system 1
                self.projector_manager.define_coordinate_system(cs_params1, False)
                cs_read = self.projector_manager.get_coordinate_system_params(name1)
                self.assertEquals(cs_read.distance, cs_params1.distance)
                self.assertEquals(cs_read.name, name1)
                # create coordinate system 2
                self.projector_manager.define_coordinate_system(cs_params2, False)
                cs_read = self.projector_manager.get_coordinate_system_params(name2)
                self.assertEquals(cs_read.distance, cs_params2.distance)
                self.assertEquals(cs_read.name, name2)
                # change to coordinate system 1 and remove 2
                self.projector_manager.set_coordinate_system(name1)
                cs_list, cs_active = self.projector_manager.get_coordinate_systems_list()
                self.assertEquals(cs_active, name1)
                self.projector_manager.remove_coordinate_system(name2)               
                pass
            else:
                self.fail("connection error")

        except Exception as e:
            self.fail("Exception raised: {}".format(e))

    # test creation of axes and frame coordinate system
    def test3_create_coordinate_system(self):
        name = "coordinate_system_test"
        # coordinate system
        cs_params            = CoordinateSystemParameters()
        cs_params.name       = name
        cs_params.distance   = 1500
        cs_params.P[0].x     = -100
        cs_params.P[0].y     = -100
        cs_params.P[1].x     =  100
        cs_params.P[1].y     = -100
        cs_params.P[2].x     =  100
        cs_params.P[2].y     =  100
        cs_params.P[3].x     = -100
        cs_params.P[3].y     =  100
        cs_params.T[0].x     =    0
        cs_params.T[0].y     =    0
        cs_params.resolution = 1000

        try:
            self.projector_manager.connect_and_setup()
            status = self.projector_manager.get_connection_status()
            if status:
                # create coordinate system
                self.projector_manager.define_coordinate_system(cs_params, False)
                cs_read = self.projector_manager.get_coordinate_system_params(name)
                self.assertEquals(cs_read.distance, cs_params.distance)
                # create axes and frame
                self.projector_manager.cs_axes_create(cs_params)
                self.projector_manager.cs_frame_create(cs_params)       
                pass
            else:
                self.fail("connection error")

        except Exception as e:
            self.fail("Exception raised: {}".format(e))            

if __name__ == '__main__':
    rosunit.unitrun('z_laser_zlp1', 'test_manager_coordinate_system', TestProjectorManager,  sysargs=None)