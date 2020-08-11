#!/usr/bin/env python3

#!/usr/bin/env python3

# Copyright (c) 2019, FADA-CATEC

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Complementary module with useful classes to support the usage of zlp library."""

class CoordinateSystemParameters:
    """This class is used as data structure with the necessary information to define a coordinate system."""
    
    def __init__(self):
        """Initialize the CoordinateSystemParameters object."""
        self.resolution = int()
        self.name       = str()
        self.d          = float()
        self.P1_x       = float()
        self.P1_y       = float()
        self.P2_x       = float()
        self.P2_y       = float()
        self.P3_x       = float()
        self.P3_y       = float()
        self.P4_x       = float()
        self.P4_y       = float()
        self.T1_x       = float()
        self.T1_y       = float()

    def set_request_params(self,cs):
        """Set the CoordinateSystemParameters values by ROS service request (CoordinateSystem.srv).
        
        Args:
            cs (object): struct with the values of the parameters, stated by the ROS service call
        """
        self.resolution = cs.resolution.data
        self.name       = cs.name_cs.data
        self.d          = cs.distance.data
        self.P1_x       = cs.p1.x
        self.P1_y       = cs.p1.y
        self.P2_x       = cs.p2.x
        self.P2_y       = cs.p2.y
        self.P3_x       = cs.p3.x
        self.P3_y       = cs.p3.y
        self.P4_x       = cs.p4.x
        self.P4_y       = cs.p4.y
        self.T1_x       = cs.T1.x
        self.T1_y       = cs.T1.y

class ProjectionElementParameters:
    """This class is used as data structure with the necessary information to create a projection element."""

    def __init__(self):
        """Initialize the ProjectionElementParameters object."""
        self.shape_type  = str()
        self.group_name  = str()
        self.shape_id    = str()
        self.x           = float()
        self.y           = float()
        self.angle       = float()
        self.length      = float()

    def set_line_params(self,proj_elem):
        """Set the ProjectionElementParameters values by ROS topic request (ProjectionElement.srv)????????????.
        
        Args:
            proj_elem (object): struct with the values of the parameters, stated by the ROS service call
        """
        self.shape_type = "polyline"
        self.group_name = proj_elem.group_name.data
        self.shape_id   = proj_elem.shape_id.data
        self.x          = proj_elem.x.data
        self.y          = proj_elem.y.data
        self.angle      = proj_elem.angle.data
        self.length     = proj_elem.length.data

    def set_request_params(self,proj_elem):
        """Set the ProjectionElementParameters values by ROS service request (ProjectionElement.srv).
        
        Args:
            proj_elem (object): struct with the values of the parameters, stated by the ROS service call
        """
        self.shape_type            = proj_elem.shape_type.data
        self.group_name = proj_elem.group_name.data
        self.shape_id              = proj_elem.shape_id.data
