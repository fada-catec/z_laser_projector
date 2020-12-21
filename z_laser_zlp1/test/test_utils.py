#!/usr/bin/env python3

import sys
import os
import unittest
import rosunit
import rospy
import socket
from copy import deepcopy

from z_laser_zlp1.zlp_utils import UserT, CoordinateSystemParameters, ProjectionElementParameters, GeometryTool
from z_laser_msgs.srv import CoordinateSystemRequest
from geometry_msgs.msg import Point

class TestUtils(unittest.TestCase):

    def test1_userT(self):

        x = 0
        y = 0
        res = 100
        h = 20
        v = 10
        try:
            user_cs = UserT(x, y, res, h, v)
            T = user_cs.T 
            self.assertEquals(T[0].x, x)
            self.assertEquals(T[0].y, y)
            self.assertEquals(T[1].x, res*h/h)
            self.assertEquals(T[2].y, res*v/h)
        except Exception as e:
            self.fail("Exception raised: {}".format(e))

    def test2_coordinate_system(self):

        cs = CoordinateSystemParameters()
        try:
            # this fields should be available
            cs.name = "test"
            cs.distance = 120.5
            cs.resolution = 200
            self.assertEquals(len(cs.P), 4)
            self.assertEquals(len(cs.T), 4)
        except Exception as e:
            self.fail("Exception raised: {}".format(e))
            
    def test3_req_conversions(self):

        req = CoordinateSystemRequest()
        req.name = "utils_test"
        req.distance = 10
        req.T0 = 5

        param = CoordinateSystemParameters()
        param.name = "utils_test_inverse"
        param.distance = 20
        p = Point(0, 2, 0)
        param.T = [p, p, p, p]

        try:
            read_param = CoordinateSystemParameters.req_to_param(req)
            self.assertEquals(read_param.name, req.name)
            self.assertEquals(read_param.distance, req.distance)
            self.assertEquals(read_param.T[0], req.T0)

            read_req = CoordinateSystemParameters.param_to_req(param)
            self.assertEquals(read_req.name, param.name)
            self.assertEquals(read_req.distance, param.distance)
            self.assertEquals(read_req.T0, param.T[0])
        except Exception as e:
            self.fail("Exception raised: {}".format(e))

    def test4_projection_element(self):

        pe = ProjectionElementParameters()
        try:
            # this fields should be available
            pe.figure_type = 0
            pe.figure_name = "pe_test"
            pe.projection_group = "pe_test_group"
            pe.position = Point(0,0,0)
            pe.size[0] = 10
            pe.size[1] = 20
            pe.angle[0] = 0
            pe.angle[1] = 0

            figure = pe.to_figure()
            self.assertEquals(figure.figure_name, pe.figure_name)
        except Exception as e:
            self.fail("Exception raised: {}".format(e))       

    def test5_geometry_tool(self):

        p1 = Point(0, 0, 0)
        p2 = Point(3, 4, 0)

        try:
            d = GeometryTool.vector_point_distance(p1, p2)
            self.assertEquals(d, 5)
            angle = GeometryTool.vector_point_angle(p1, p2)
            self.assertAlmostEqual(angle, 53.13, 3)
        except Exception as e:
            self.fail("Exception raised: {}".format(e))                      

if __name__ == '__main__':
    rosunit.unitrun('z_laser_zlp1', 'test_utils', TestUtils,  sysargs=None)