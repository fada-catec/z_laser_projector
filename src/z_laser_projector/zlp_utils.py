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

"""Complementary module with useful classes to support the usage of zlp library."""

class ProjectionPoint:
    def __init__(self,x,y):
        self.x = x
        self.y = y


class CoordinateSystemParameters:
    """This class is used as data object with the necessary parameters to define a coordinate system.

    Attributes:
        name (str): coordinate system name
        d (float): distance between the projection surface and the projector device
        res (int): resolution of user reference system {T}
        P1 (object): object with the x,y position of point p1 from projector reference system {P}
        P2 (object): object with the x,y position of point p2 from projector reference system {P}
        P3 (object): object with the x,y position of point p3 from projector reference system {P}
        P4 (object): object with the x,y position of point p4 from projector reference system {P}
        T1 (object): object with the x,y position of point T1 from user reference system {T}
        T2 (object): object with the x,y position of point T2 from user reference system {T}
        T3 (object): object with the x,y position of point T3 from user reference system {T}
        T4 (object): object with the x,y position of point T4 from user reference system {T}
    """
    def __init__(self):
        """Initialize the CoordinateSystemParameters object."""
        self.name = str()
        self.d    = float()
        self.res  = int()
        self.P1   = ProjectionPoint(0,0)
        self.P2   = ProjectionPoint(0,0)
        self.P3   = ProjectionPoint(0,0)
        self.P4   = ProjectionPoint(0,0)
        self.T1   = ProjectionPoint(0,0)
        self.T2   = ProjectionPoint(0,0)
        self.T3   = ProjectionPoint(0,0)
        self.T4   = ProjectionPoint(0,0)

    def set_request_params(self,cs):
        """Set the CoordinateSystemParameters values by ROS service request (CoordinateSystem.srv).
        
        Args:
            cs (object): object with the parameters to set the class attributes values, stated by the ROS service call
        """
        self.name = cs.name_cs.data
        self.d    = cs.distance.data
        self.res  = cs.resolution.data
        self.P1   = ProjectionPoint(cs.p1.x, cs.p1.y)
        self.P2   = ProjectionPoint(cs.p2.x, cs.p2.y)
        self.P3   = ProjectionPoint(cs.p3.x, cs.p3.y)
        self.P4   = ProjectionPoint(cs.p4.x, cs.p4.y)
        self.T1   = ProjectionPoint(cs.T1.x, cs.T1.y)
        

class ProjectionElementParameters:
    """This class is used as data object with the necessary information to create a projection element.
    
    Attributes:
        shape_type (str): type of projection element (polyline, circle, ...)
        group_name (str): name of the projection group to which the projection element belongs
        shape_id (str): name of the projection element to define
        x (float): x-position of the projection element's 'characteristic point'. 
        The 'characteristic point' for a polyline refers to the starting point
        y (float): y-position of polyline starting point
        angle (float): polyline slope
        length (float): polyline length
    """
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
        """Set the ProjectionElementParameters values by ROS topic request (projector_zlp1/add_line).
        
        Args:
            proj_elem (object): object with the parameters values to set at the class attributes, 
            stated by the ROS topic pub
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
            proj_elem (object): object with the parameters values to set at the class attributes values, 
            stated by the ROS service call
        """
        self.shape_type = proj_elem.shape_type.data
        self.group_name = proj_elem.group_name.data
        self.shape_id   = proj_elem.shape_id.data
