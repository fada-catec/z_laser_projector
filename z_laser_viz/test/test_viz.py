#!/usr/bin/env python3

import sys
import os
import unittest
import rosunit
import rospy
import socket

from z_laser_viz.zlp_viz import ZLPVisualizer
from z_laser_zlp1.zlp_utils import  CoordinateSystemParameters
from z_laser_msgs.srv import ProjectionElementRequest
from z_laser_msgs.msg import Figure
from visualization_msgs.msg import MarkerArray

class TestViz(unittest.TestCase):

    # test creation of base marker
    def test1_create_maker(self):

        viz = ZLPVisualizer()

        try:
            m = viz.base_marker("test_marker")
            self.assertEquals(m.header.frame_id, "test_marker")
        except Exception as e:
            self.fail("%s" %e)

    # test creation of axis and frame for coordinate system
    def test2_create_frame(self):

        viz = ZLPVisualizer()

        cs_params = CoordinateSystemParameters()
        cs_params.P[0].x = -100
        cs_params.P[0].y = -100
        cs_params.P[1].x =  100
        cs_params.P[1].y = -100
        cs_params.P[2].x =  100
        cs_params.P[2].y =  100
        cs_params.P[3].x = -100
        cs_params.P[3].y =  100

        try:
            axis_x, axis_y = viz.coord_sys_axes(cs_params)
            frame = viz.coord_sys_frame(cs_params)
            self.assertEquals(len(axis_x.points), 2)
            self.assertEquals(len(axis_y.points), 2)
            self.assertEquals(len(frame.points), 5)
        except Exception as e:
            self.fail("%s" %e)

    # test define figure equations
    def test3_figure_eq(self):

        viz = ZLPVisualizer()

        try:
            line = viz.line_eq(10, 0)
            oval = viz.oval_eq(5, 2, 0)
            circle = viz.circle_eq(5, 0, 360)
        except Exception as e:
            self.fail("%s" %e)

    # test translate and rotate
    def test4_operations(self):

        viz = ZLPVisualizer()
        m = viz.base_marker("test_marker")

        figure = Figure()
        figure.position.x = 0      
        figure.position.y = 0        
        figure.position.z = 0        
        figure.size.append(100) # length
        figure.size.append(10) # height
        figure.angle.append(0)
        figure.angle.append(0)

        try: 
            viz.translate(m, dx=1)
            self.assertEquals(m.pose.position.x, 1)
            self.assertEquals(m.pose.position.y, 0)
            viz.translate(m, dy=2)
            self.assertEquals(m.pose.position.x, 1)
            self.assertEquals(m.pose.position.y, 2)
            viz.translate(m, dy=-1)
            self.assertEquals(m.pose.position.y, 1)
            viz.rotate(m, 90)
            figure.figure_type = Figure.POLYLINE
            viz.scale(m, 2, figure)
            figure.figure_type = Figure.CIRCLE
            viz.scale(m, 2, figure)
            figure.figure_type = Figure.ARC
            viz.scale(m, 2, figure)
            figure.figure_type = Figure.OVAL
            viz.scale(m, 2, figure)
            figure.figure_type = Figure.TEXT
            viz.scale(m, 2, figure)

        except Exception as e:
            self.fail("%s" %e)

if __name__ == '__main__':
    rosunit.unitrun('z_laser_viz', 'test_viz', TestViz,  sysargs=None)