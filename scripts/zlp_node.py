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

"""Node that provides a collection of services that allows to operate and control the ZLP1 laser projector and simplify the task of 
developing further advanced features."""

import rospy
import rospkg

from z_laser_projector.zlp_projector_ros import ZLPProjectorROS

if __name__ == '__main__':
    rospy.init_node('zlp_node')

    projector_IP    = rospy.get_param('projector_IP', "192.168.10.10") 
    server_IP       = rospy.get_param('server_IP', "192.168.10.11") 
    connection_port = rospy.get_param('connection_port', 9090) 
    license_file    = rospy.get_param('license_file', "1900027652.lic") 

    # define license file path
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('z_laser_projector')
    license_path = pkg_path + "/lic/" + license_file

    # create node instance
    zlp_node = ZLPProjectorROS(projector_IP, server_IP, connection_port, license_path)

    # connect, load license and activate projector
    error = zlp_node.setup_projector()

    # if set by user, create a coordinate system
    if not error:
        zlp_node.initialize_coordinate_system()

    # create services to interact with projector
    zlp_node.open_services()

    # create handler to close connection when exiting node
    rospy.on_shutdown(zlp_node.shutdown_handler)

    rospy.spin()