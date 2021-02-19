#!/usr/bin/env python3

# Copyright (c) 2021, FADA-CATEC

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import rospy
import rospkg
import ezdxf

import z_laser_zlp1.zlp_dxf as zlp_dxf
from z_laser_msgs.msg import Figure

if __name__ == '__main__':
    node_name = 'zlp_dxf_reader'
    rospy.init_node(node_name)

    # get dxf file path
    dxf_file_name = rospy.get_param(node_name + '/dxf_file', default="dxf_test")
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('z_laser_zlp1')
    dxf_file = pkg_path + "/dxf/" + dxf_file_name + ".dxf"

    # create publisher
    pub = rospy.Publisher('add_projection_element', Figure, queue_size=10)
    rospy.sleep(0.1)
    if pub.get_num_connections() == 0:
        rospy.logerr("No subscriber found for %s", pub.name)
        sys.exit()

    # read and publish figures from dxf file
    try:
        doc = ezdxf.readfile(dxf_file)
        msp = doc.modelspace()

        for i, elem in enumerate(msp):
            
            proj_elem_params = zlp_dxf.get_params(elem, dxf_file_name, elem.dxftype(), i)
            rospy.loginfo("[%s] loaded figure" % elem.dxftype()) 

            pub.publish(proj_elem_params)

    except TypeError as e:
        rospy.logwarn(e)

    except IOError as e:
        rospy.logerr(e) 

    except ezdxf.DXFStructureError as e:
        rospy.logerr(e) 