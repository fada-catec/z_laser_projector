#!/usr/bin/env python3

from std_msgs.msg import Bool, String, Float64, Int16

class CoordinateSystemParameters:
    """This class is used as data structure with the necessary information to define a coordinate system."""
    
    def __init__(self):
        """Initialize the CoordinateSystemParameters object."""
        self.name         = String()
        self.d            = Float64()
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
        self.scale_factor = Int16()

    def set_params(self,cs):
        """Set the CoordinateSystemParameters values.
        
            Args:
                struct cs: struct with the necessary parameters to create the coordinate system"""
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
        self.scale_factor = cs.scale_factor.data

class ProjectionElementParameters:
    """This class is used as data structure with the necessary information to create a projection element."""

    def __init__(self):
        """Initialize the ProjectionElementParameters object."""
        self.shape_type            = String()
        self.projection_group_name = String()
        self.shape_id              = String()
        self.x                     = Float64()
        self.y                     = Float64()
        self.angle                 = Float64()
        self.length                = Float64()

    def set_params(self,proj_elem):
        """Set the ProjectionElementParameters values.
        
            Args:
                struct proj_elem: struct with the necessary parameters to create the projection element"""
        self.shape_type            = proj_elem.shape_type.data
        self.projection_group_name = proj_elem.projection_group_name.data
        self.shape_id              = proj_elem.shape_id.data
        self.x                     = proj_elem.x.data
        self.y                     = proj_elem.y.data
        self.angle                 = proj_elem.angle.data
        self.length                = proj_elem.length.data