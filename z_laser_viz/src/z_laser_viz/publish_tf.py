#! /usr/bin/env python

import rospy
import tf
import math

from z_laser_msgs.srv import CoordinateSystem, CoordinateSystemResponse
from z_laser_msgs.srv import CoordinateSystemName, CoordinateSystemNameResponse
from z_laser_msgs.srv import CoordinateSystemShow, CoordinateSystemShowResponse
from z_laser_msgs.srv import CoordinateSystemList, CoordinateSystemListResponse
from z_laser_msgs.srv import ProjectionElement, ProjectionElementResponse

class ProjectionCoordinatesSystemsTF(object):

    def __init__(self):

        # self.manual_cs = rospy.Service('define_coordinate_system', CoordinateSystem, self.manual_define_coord_sys_cb)
        # self.set_cs    = rospy.Service('set_coordinate_system', CoordinateSystemName, self.set_coord_sys_cb)
        # self.rem_cs    = rospy.Service('remove_coordinate_system', CoordinateSystemName, self.remove_coord_sys_cb)
        # self.show_cs   = rospy.Service('show_active_coordinate_system', CoordinateSystemShow, self.show_coord_sys_cb)

        self.br_P = tf.TransformBroadcaster()
        self.br_T = tf.TransformBroadcaster()

        self.cs_name = ""
        
        # self.d = None
        # self.P_x = None
        # self.P_y = None


    def get_param(self):

        self.cs_name = rospy.get_param('coordinate_system_name', "")
        self.d       = rospy.get_param('coordinate_system_distance', 1500) * 0.001
        self.P_x     = rospy.get_param('P1/x', -100) * 0.001
        self.P_y     = rospy.get_param('P1/y', -100) * 0.001
        # self.T_x     = rospy.get_param('T1/x',    0) * 0.001
        # self.T_y     = rospy.get_param('T1/y',    0) * 0.001

        # self.P_x     = self.P_x - self.T_x
        # self.P_y     = self.P_y - self.T_y


if __name__ == '__main__':

    rospy.init_node('tf_broadcaster')

    cs_tf = ProjectionCoordinatesSystemsTF()

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():

        cs_tf.get_param()

        cs_tf.br_P.sendTransform((0, 0, cs_tf.d),
                    tf.transformations.quaternion_from_euler(math.pi, 0, 0),
                     rospy.Time.now(),
                     "[P]",
                     "zlp1_link")

        if cs_tf.cs_name:
            cs_tf.br_T.sendTransform((cs_tf.P_x, cs_tf.P_y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        cs_tf.cs_name,
                        "[P]")

        rate.sleep()

    rospy.spin()