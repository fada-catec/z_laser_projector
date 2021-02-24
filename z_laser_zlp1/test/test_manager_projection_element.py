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

    # test creation of projection elements
    def test1_create_projection_elements(self):
        # coordinate system
        cs_params            = CoordinateSystemParameters()
        cs_params.name       = "coordinate_system_test"
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
        
        # common
        figure = Figure()
        figure.projection_group = "figure_test"
        figure.position.x = 0      
        figure.position.y = 0        
        figure.position.z = 0        
        figure.angle.append(0)
        # polyline
        polyline = deepcopy(figure)
        polyline.figure_type = Figure.POLYLINE
        polyline.figure_name = "polyline_test"
        polyline.size.append(100) # length
        # circle
        circle = deepcopy(figure)
        circle.figure_type = Figure.CIRCLE
        circle.figure_name = "circle_test"
        circle.size.append(50) # radius
        # text
        text = deepcopy(figure)
        text.figure_type = Figure.TEXT
        text.figure_name = "text_test"
        text.size.append(10) # heigh
        text.size.append(2) # char spacing
        text.text = "TEST"

        try:
            self.projector_manager.connect_and_setup()
            status = self.projector_manager.get_connection_status()
            if status:
                self.projector_manager.define_coordinate_system(cs_params, False)
                self.projector_manager.create_polyline(polyline)
                self.projector_manager.create_curve(circle)
                self.projector_manager.create_text(text)
                pass
            else:
                self.fail("connection error")

        except Exception as e:
            self.fail("Exception raised: {}".format(e))

    # test write and read data of projection element
    def test2_projection_element(self):    
        # coordinate system
        cs_params            = CoordinateSystemParameters()
        cs_params.name       = "coordinate_system_test"
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

        # polyline
        polyline = Figure()
        polyline.figure_type = Figure.POLYLINE
        polyline.projection_group = "figure_test"
        polyline.figure_name = "polyline_test"
        polyline.position.x = 0.0      
        polyline.position.y = 0.0        
        polyline.position.z = 0.0        
        polyline.angle.append(0)
        polyline.size.append(100) # length

        # proj_elem
        proj_elem = ProjectionElementRequest()
        proj_elem.figure_type = Figure.POLYLINE
        proj_elem.projection_group = "figure_test"
        proj_elem.figure_name = "polyline_test"

        try:
            self.projector_manager.connect_and_setup()
            status = self.projector_manager.get_connection_status()
            if status:
                self.projector_manager.define_coordinate_system(cs_params, False)
                self.projector_manager.create_polyline(polyline)
                figure = self.projector_manager.get_proj_elem(proj_elem)
                self.assertEqual(figure.size[0], polyline.size[0])
                self.assertEqual(figure.angle[0], polyline.angle[0])
                self.assertEqual(figure.position.x, polyline.position.x)
                # if figure is ok, do operations
                self.projector_manager.hide_proj_elem(proj_elem)
                self.projector_manager.unhide_proj_elem(proj_elem)
                self.projector_manager.remove_proj_elem(proj_elem)
            else:
                self.fail("connection error")

        except Exception as e:
            self.fail("Exception raised: {}".format(e))

    # test send and read data of pointer projection element
    def test3_define_pointer(self):
        
        # coordinate system
        cs_params            = CoordinateSystemParameters()
        cs_params.name       = "coordinate_system_test"
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

        # pointer
        pointer = Figure()
        pointer.figure_type = Figure.POINTER
        pointer.projection_group = "figure_test"
        pointer.figure_name = "pointer_test"
        pointer.position.x = 0.0      
        pointer.position.y = 0.0        
        pointer.position.z = 0.0
        pointer.size.append(5) # height
        pointer.angle.append(0)

        # proj_elem
        proj_elem = ProjectionElementRequest()
        proj_elem.figure_type = Figure.POINTER
        proj_elem.projection_group = "figure_test"
        proj_elem.figure_name = "pointer_test"

        try:
            self.projector_manager.connect_and_setup()
            status = self.projector_manager.get_connection_status()
            if status:
                self.projector_manager.define_coordinate_system(cs_params, False)
                self.projector_manager.create_pointer(pointer)
                figure = self.projector_manager.get_proj_elem(proj_elem)
                self.assertEqual(figure.size[0], pointer.size[0])
                self.assertEqual(figure.position.x, pointer.position.x)
            else:
                self.fail("connection error")

        except Exception as e:
            self.fail("Exception raised: {}".format(e))

if __name__ == '__main__':
    rosunit.unitrun('z_laser_zlp1', 'test_manager_projection_element', TestProjectorManager,  sysargs=None)