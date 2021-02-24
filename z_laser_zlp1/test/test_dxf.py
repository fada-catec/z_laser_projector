#!/usr/bin/env python3

import sys
import os
import unittest
import rosunit
import rospy
import rospkg
import ezdxf

from z_laser_msgs.msg import Figure
import z_laser_zlp1.zlp_dxf as zlp_dxf

class TestDxf(unittest.TestCase):

    # test exception is thrown when no valid type is specified
    def test1_no_shape_type(self):

        try:
            # insert is not a valid type
            zlp_dxf.get_params(None, "test", "INSERT", 0)
            self.fail("error test1")
        
        except TypeError:
            pass 

    # test every shape presented in example_dxf
    def test2_shapes(self):

        # get dxf file path
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('z_laser_zlp1')
        dxf_file = pkg_path + "/dxf/dxf_test.dxf"

        shape_types = [Figure.TEXT, Figure.POLYLINE, Figure.CIRCLE, Figure.OVAL, Figure.ARC]

        try:
            doc = ezdxf.readfile(dxf_file)
            msp = doc.modelspace()
    
            for i, elem in enumerate(msp):
                params = zlp_dxf.get_params(elem, "dxf_test", elem.dxftype(), i)
                self.assertEqual(shape_types[i], params.figure_type)
                if params.figure_type == (Figure.CIRCLE or Figure.OVAL or Figure.ARC):
                    self.assertEqual(params.position.x, 200)
                    self.assertEqual(params.position.y, 200)
                if params.figure_type == Figure.POLYLINE:
                    self.assertEqual(params.position.x, 0)
                    self.assertEqual(params.position.y, 0)                    
                self.assertEqual(params.position.z, 0)

        except TypeError:
            self.fail("error test2")

if __name__ == '__main__':
    rosunit.unitrun('z_laser_zlp1', 'test_dxf', TestDxf, sysargs=None)