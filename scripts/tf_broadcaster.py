#!/usr/bin/env python3

# Copyright (c) 2020, FADA-CATEC

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import tf
import math

class ProjectionCoordinatesSystemsTF(object):

    def __init__(self):

        self.d = None
        self.P_x = None
        self.P_y = None

    def get_cs_params(self):
        
        self.cs_name = rospy.get_param('coordinate_system_name', "default_cs")
        self.d       = rospy.get_param('coordinate_system_distance', 1500) * 0.001
        self.P_x     = rospy.get_param('P1/x', -100) * 0.001
        self.P_y     = rospy.get_param('P1/y', -100) * 0.001
        self.T_x     = rospy.get_param('T1/x',    0) * 0.001
        self.T_y     = rospy.get_param('T1/y',    0) * 0.001

        self.P_x     = self.P_x - self.T_x
        self.P_y     = self.P_y - self.T_y

if __name__ == '__main__':

    rospy.init_node('tf_broadcaster')

    projection_cs_tf = ProjectionCoordinatesSystemsTF()

    br_P = tf.TransformBroadcaster()
    br_T = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():

        projection_cs_tf.get_cs_params()

        br_P.sendTransform((0, 0, projection_cs_tf.d),
                     tf.transformations.quaternion_from_euler(math.pi/2, 0, 0),
                     rospy.Time.now(),
                     "[P]",
                     "zlp1_link")

        br_T.sendTransform((projection_cs_tf.P_x, projection_cs_tf.P_y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     projection_cs_tf.cs_name,
                     "[P]")

        rate.sleep()

    rospy.spin()