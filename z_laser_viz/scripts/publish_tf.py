#! /usr/bin/env python

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

def get_ros_params(ns):

    cs_name = rospy.get_param(ns + '/coordinate_system_name', "")
    d       = rospy.get_param(ns + '/coordinate_system_distance', 1500) * 0.001
    Px      = rospy.get_param(ns + '/P0/x', -100) * 0.001
    Py      = rospy.get_param(ns + '/P0/y', -100) * 0.001

    return cs_name,d,Px,Py


if __name__ == '__main__':

    rospy.init_node('tf_broadcaster')

    broadcast_P = tf.TransformBroadcaster()
    broadcast_T = tf.TransformBroadcaster()

    ns = rospy.get_param('tf_broadcaster/namespace', "zlaser")

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():

        cs_name,dist,Px,Py = get_ros_params(ns)

        broadcast_P.sendTransform((0, 0, dist),
                    tf.transformations.quaternion_from_euler(math.pi, 0, 0),
                     rospy.Time.now(),
                     "[P]",
                     "zlp1_link")

        if cs_name:
            broadcast_T.sendTransform((Px, Py, 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        cs_name,
                        "[P]")

        rate.sleep()

    rospy.spin()