#! /usr/bin/env python3

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

from z_laser_viz.zlp_viz import ZLPVisualizer
from visualization_msgs.msg import MarkerArray

if __name__ == '__main__':

    rospy.init_node('zlp_viz_node')

    zlp_visualizer = ZLPVisualizer()
    zlp_visualizer.open_services()

    cs_markers_pub = rospy.Publisher('coord_sys_markers', MarkerArray, queue_size=10)
    pe_markers_pub = rospy.Publisher('proj_elem_markers', MarkerArray, queue_size=10)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        cs_markers_pub.publish(zlp_visualizer.cs_marker_array)
        pe_markers_pub.publish(zlp_visualizer.pe_marker_array)
        rate.sleep()

    rospy.spin()
