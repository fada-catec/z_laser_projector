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
        self.name         = str()
        self.d            = float()
        self.x1           = float()
        self.y1           = float()
        self.x2           = float()
        self.y2           = float()
        self.x3           = float()
        self.y3           = float()
        self.x4           = float()
        self.y4           = float()
        self.T1_x         = float()
        self.T1_y         = float()
        self.resolution = int()

    def set_request_params(self,cs):
        """Set the CoordinateSystemParameters values by ROS service request (CoordinateSystem.srv).
        
        Args:
            cs (object): struct with the values of the parameters, stated by the ROS service call
        """
        self.name         = cs.name_cs.data
        self.d            = cs.distance.data
        self.x1           = cs.p1.x
        self.y1           = cs.p1.y
        self.x2           = cs.p2.x
        self.y2           = cs.p2.y
        self.x3           = cs.p3.x
        self.y3           = cs.p3.y
        self.x4           = cs.p4.x
        self.y4           = cs.p4.y
        self.T1_x         = cs.T1.x
        self.T1_y         = cs.T1.y
        self.resolution = cs.resolution.data

class ProjectionElementParameters:
    """This class is used as data structure with the necessary information to create a projection element."""

    def __init__(self):
        """Initialize the ProjectionElementParameters object."""
        self.shape_type            = str()
        self.projection_group_name = str()
        self.shape_id              = str()
        self.x                     = float()
        self.y                     = float()
        self.angle                 = float()
        self.length                = float()

    def set_line_params(self,proj_elem):
        """Set the ProjectionElementParameters values by ROS topic request (ProjectionElement.srv)????????????.
        
        Args:
            proj_elem (object): struct with the values of the parameters, stated by the ROS service call
        """
        self.shape_type            = "polyline"
        self.projection_group_name = proj_elem.projection_group_name.data
        self.shape_id              = proj_elem.shape_id.data
        self.x                     = proj_elem.x.data
        self.y                     = proj_elem.y.data
        self.angle                 = proj_elem.angle.data
        self.length                = proj_elem.length.data

    def set_request_params(self,proj_elem):
        """Set the ProjectionElementParameters values by ROS service request (ProjectionElement.srv).
        
        Args:
            proj_elem (object): struct with the values of the parameters, stated by the ROS service call
        """
        self.shape_type            = proj_elem.shape_type.data
        self.projection_group_name = proj_elem.projection_group_name.data
        self.shape_id              = proj_elem.shape_id.data
