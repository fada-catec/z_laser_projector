#! /usr/bin/env python

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