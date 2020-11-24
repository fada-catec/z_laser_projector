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

"""This module contains utility classes and methods which ease the management and operation of projection elements."""

import os
import sys
from math import sin, cos, pi, radians
import copy

from z_laser_msgs.msg import Figure

from z_laser_zlp1.zlp_utils import ProjectionElementParameters, GeometryTool
# from zlp_utils import ProjectionElementParameters, GeometryTool


class ProjectionElement(object):
    """This class implement the functions related with projection elements.
    
    Args:
        module_id (str): function module identification name
        thrift_client (object): object with the generated client to communicate with the projector

    Attributes:
        module_id (str): function module identification name
        figures_list (list): list with the figures' identificator names
        default_projection_element (object): basic object initialization for Projection Elements
    """
    def __init__(self, module_id, thrift_client):
        """Initialize the ProjectionElement object."""
        self.__thrift_client = thrift_client
        self.__geometry_tool = GeometryTool(thrift_client)

        self.module_id = module_id
        self.figures_list = ProjectionElementParameters().figures_list

        self.default_projection_element = self.__thrift_client.thrift_interface.ProjectionElement()
        self.default_projection_element.pen = 0
        self.default_projection_element.coordinateSystemList = []
        self.default_projection_element.projectorIDList = []
        self.default_projection_element.userTrans = self.__geometry_tool.create_matrix4x4()
        self.default_projection_element.activated = True

    def init_projection_element(self, elem):
        """Initialize new projection element.

        Args:
            elem (object): projection element object to initialize
            
        Returns:
            object: projection element object with fields initialized
        """
        elem.coordinateSystemList = copy.deepcopy(self.default_projection_element.coordinateSystemList)
        elem.projectorIDList = copy.deepcopy(self.default_projection_element.projectorIDList)
        elem.userTrans = copy.deepcopy(self.default_projection_element.userTrans)
        elem.activated = self.default_projection_element.activated
        elem.pen = self.default_projection_element.pen
        return elem

    def create_polyline(self, name):
        """Create and initialize a new polyline object.
            
        Args:
            name (str): polyline name
        
        Returns:
            object: polyline struct with fields initialized
        """
        polyline = self.__thrift_client.thrift_interface.Polyline()
        polyline = self.init_projection_element(polyline)
        polyline.name = name
        polyline.polylineList = []
        return polyline

    def define_polyline(self, cs_name, proj_elem_params):
        """Create a new line as projection element.

        Args:
            cs_name (str): name of coordinate system which the new projection element will be added
            proj_elem_params (object): object with the parameters to identify and define a line as a new projection element
                
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            group  = proj_elem_params.projection_group
            id     = proj_elem_params.figure_name
            x      = proj_elem_params.position.x
            y      = proj_elem_params.position.y
            angle  = proj_elem_params.angle[0]
            length = proj_elem_params.size[0]

            polyline_name = group + "/polyline/" + id
            polyline = self.create_polyline(polyline_name)

            linestring = [ self.__geometry_tool.create_3d_point(x, y),
                           self.__geometry_tool.create_3d_point(x+length*cos(angle*pi/180), y+length*sin(angle*pi/180))]
            
            polyline.polylineList = [linestring]
            polyline.activated = True
            polyline.coordinateSystemList = [cs_name]
        
            self.__thrift_client.SetPolyLine(polyline)
            
            success = True
            message = polyline_name + " polyline created at [" + cs_name + "] coordinate system."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def define_arrow(self, cs_name, proj_elem_params):
        """Create a new arrow as projection element.

        Args:
            cs_name (str): name of coordinate system which the new projection element will be added
            proj_elem_params (object): object with the parameters to identify and define an arrow as a new projection element
                
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            group = proj_elem_params.projection_group
            id    = proj_elem_params.figure_name
            
            arrow_name = group + "/polyline/" + id
            arrow = self.create_polyline(arrow_name)
            
            angle = proj_elem_params.angle[0]
            length = proj_elem_params.size[0]

            x1 = proj_elem_params.position.x
            y1 = proj_elem_params.position.y

            x2 = x1 + length*cos(radians(angle))
            y2 = y1 + length*sin(radians(angle))
            
            angle += 160
            x3 = x2 + length/4*cos(radians(angle))
            y3 = y2 + length/4*sin(radians(angle))
            
            angle += 40
            x4 = x2 + length/4*cos(radians(angle))
            y4 = y2 + length/4*sin(radians(angle))
            
            linestring = [ self.__geometry_tool.create_3d_point(x1, y1),
                           self.__geometry_tool.create_3d_point(x2, y2),
                           self.__geometry_tool.create_3d_point(x3, y3),
                           self.__geometry_tool.create_3d_point(x4, y4),
                           self.__geometry_tool.create_3d_point(x2, y2)]
            
            arrow.polylineList = [linestring]
            arrow.activated = True
            arrow.coordinateSystemList = [cs_name]
            
            self.__thrift_client.SetPolyLine(arrow)
            
            success = True
            message = arrow_name + " arrow created at [" + cs_name + "] coordinate system."
        
        except Exception as e:
            success = False
            message = e
        
        return success,message

    def define_rectangle(self, cs_name, proj_elem_params, points):
        """Create a new rectangle as projection element.

        Args:
            cs_name (str): name of coordinate system which the new projection element will be added
            proj_elem_params (object): object with the parameters to identify and define a rectangle as a new projection element
            points (list): 4 points list of rectangle corners 
                
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            group  = proj_elem_params.projection_group
            id     = proj_elem_params.figure_name

            rectangle_name = group + "/polyline/" + id
            rectangle = self.create_polyline(rectangle_name)

            linestring = []
            for i in [0, 1, 2, 3, 0]:
                linestring.append(self.__geometry_tool.create_3d_point(points[i].x, points[i].y))
                            
            rectangle.polylineList = [linestring]
            rectangle.activated = True
            rectangle.coordinateSystemList = [cs_name]
            
            self.__thrift_client.SetPolyLine(rectangle)
            
            success = True
            message = rectangle_name + " rectangle created at [" + cs_name + "] coordinate system."
        
        except Exception as e:
            success = False
            message = e
        
        return success,message
        
    def create_curve(self, name, curve_type):
        """Create and initialize a new curve (circle, arc or oval) object.
            
        Args:
            name (str): curve name
        
        Returns:
            curve (object): curve object with fields initialized
        """
        if curve_type == "circle" or curve_type == "arc":
            curve = self.__thrift_client.thrift_interface.CircleSegment()
        elif curve_type == "oval":
            curve = self.__thrift_client.thrift_interface.OvalSegment()
        curve = self.init_projection_element(curve)
        curve.name = name
        return curve

    def define_circle(self, cs_name, proj_elem_params):
        """Define a new circle as projection element.

        Args:
            cs_name (str): name of coordinate system which the new projection element will be added
            proj_elem_params (object): object with the necessary parameters to identify and define a circle as a 
            new projection element
                
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            group    = proj_elem_params.projection_group
            id       = proj_elem_params.figure_name
            center_x = proj_elem_params.position.x
            center_y = proj_elem_params.position.y
            radius   = proj_elem_params.size[0]

            circle_name = group + "/circle/" + id
            circle = self.create_curve(circle_name,"circle")
            circle.radius = radius
            circle.center = self.__geometry_tool.create_3d_point(center_x, center_y)
            circle.activated = True
            circle.coordinateSystemList = [cs_name]

            self.__thrift_client.SetCircleSegment(circle)
            
            success = True
            message = circle_name + " circle created at [" + cs_name + "] coordinate system."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def define_arc(self, cs_name, proj_elem_params):
        """Create a new arc as projection element.

        Args:
            cs_name (str): name of coordinate system which the new projection element will be added
            proj_elem_params (object): object with the necessary parameters to identify and define an arc as a 
            new projection figure
                
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            group       = proj_elem_params.projection_group
            id          = proj_elem_params.figure_name
            center_x    = proj_elem_params.position.x
            center_y    = proj_elem_params.position.y
            radius      = proj_elem_params.size[0]
            start_angle = proj_elem_params.angle[0]
            end_angle   = proj_elem_params.angle[1]

            arc_name = group + "/arc/" + id
            arc = self.create_curve(arc_name,"arc")
            arc.radius = radius
            arc.center = self.__geometry_tool.create_3d_point(center_x, center_y)
            arc.startAngle = start_angle
            arc.endAngle = end_angle
            arc.activated = True
            arc.coordinateSystemList = [cs_name]

            self.__thrift_client.SetCircleSegment(arc)
            
            success = True
            message = arc_name + " arc created at [" + cs_name + "] coordinate system."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def define_oval(self, cs_name, proj_elem_params):
        """Create a new oval as projection element.

        Args:
            cs_name (str): name of coordinate system which the new projection element will be added
            proj_elem_params (object): object with the necessary parameters to identify and define an oval as a 
            new projection figure
                
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            group    = proj_elem_params.projection_group
            id       = proj_elem_params.figure_name
            center_x = proj_elem_params.position.x
            center_y = proj_elem_params.position.y
            angle    = proj_elem_params.angle[0]
            width    = proj_elem_params.size[0]*2
            height   = proj_elem_params.size[1]*2

            oval_name = group + "/oval/" + id
            oval = self.create_curve(oval_name,"oval")
            oval.width = width
            oval.height = height
            oval.center = self.__geometry_tool.create_3d_point(center_x, center_y)
            oval.angle = angle
            oval.activated = True
            oval.coordinateSystemList = [cs_name]

            self.__thrift_client.SetOvalSegment(oval)
            
            success = True
            message = oval_name + " oval created at [" + cs_name + "] coordinate system."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def create_text(self, name):
        """Create and initialize a new text object.
            
        Args:
            name (str): text object name
        
        Returns:
            text (object): text object with fields initialized
        """
        text = self.__thrift_client.thrift_interface.TextElement()
        text = self.init_projection_element(text)
        text.name = name
        return text

    def define_text(self, cs_name, proj_elem_params):
        """Create a new text as projection element.

        Args:
            cs_name (str): name of coordinate system which the new projection element will be added
            proj_elem_params (object): object with the necessary parameters to identify and define a text as a 
            new projection figure
                
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            group        = proj_elem_params.projection_group
            id           = proj_elem_params.figure_name
            x_position   = proj_elem_params.position.x
            y_position   = proj_elem_params.position.y
            angle        = proj_elem_params.angle[0]
            height       = proj_elem_params.size[0]
            char_spacing = proj_elem_params.size[1]
            text_proj    = proj_elem_params.text

            text_name = group + "/text/" + id
            text = self.create_text(text_name)
            text.text = text_proj
            text.charSpacing = char_spacing
            text.position = self.__geometry_tool.create_3d_point(x_position, y_position)
            text.angle = angle
            text.height = height
            text.activated = True
            text.coordinateSystemList = [cs_name]

            self.__thrift_client.SetTextElement(text)
            
            success = True
            message = text_name + " text created at [" + cs_name + "] coordinate system."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def get_figure(self, params):
        """Get properties of a defined projection element.

        Args:
            params (object): object with the necessary parameters to identify the projection element

        Returns:
            tuple[object, bool, str]: the first value in the returned tuple is the object of the projection element,
            the second is a bool success value and the third is an information message string
        """
        try:
            figure_type = params.figure_type
            group       = params.projection_group
            id          = params.figure_name
            
            name = group + self.figures_list[figure_type] + id

            proj_elem = ProjectionElementParameters()
            proj_elem.projection_group = group
            proj_elem.figure_name      = id
            proj_elem.figure_type      = figure_type

            if figure_type == Figure.POLYLINE:
                figure = self.get_polyline(name,proj_elem)
            elif figure_type == Figure.CIRCLE:
                figure = self.get_circle(name,proj_elem)
            elif figure_type == Figure.ARC:
                figure = self.get_arc(name,proj_elem)
            elif figure_type == Figure.OVAL:
                figure = self.get_oval(name,proj_elem)
            elif figure_type == Figure.TEXT:
                figure = self.get_text(name,proj_elem)
            else:
                success = False
                message = "Figure " + name + "does not exist."

            if figure:
                success = True
                message = "Projection element get correct."
            else:
                success = False
                message = "Projection element get error."

        except Exception as e:
            success = False 
            message = e
            figure = []

        return figure,success,message
    
    def get_polyline(self, name, proj_elem):
        """Get properties of a defined polyline.

        Args:
            name (str): name of the polyline
            params (object): object to fill with the projection element properties

        Returns:
            object: object with the properties of the polyline
        """
        polyline = self.__thrift_client.GetPolyLine(name)
        if polyline:
            start_point = polyline.polylineList[0][0]
            end_point = polyline.polylineList[0][1]

            angle = GeometryTool.vector_point_angle(start_point, end_point)
            length = GeometryTool.vector_point_distance(start_point, end_point)

            proj_elem.position.x = start_point.x
            proj_elem.position.y = start_point.y
            proj_elem.angle[0]   = angle
            proj_elem.size[0]    = length
            return proj_elem
        else:
            return []

    def get_circle(self, name, proj_elem):
        """Get properties of a defined circle.

        Args:
            name (str): name of the circle
            params (object): object to fill with the projection element properties

        Returns:
            object: object with the properties of the polyline
        """
        circle = self.__thrift_client.GetCircleSegment(name)
        if circle:
            proj_elem.position.x = circle.center.x
            proj_elem.position.y = circle.center.y
            proj_elem.size[0]    = circle.radius
            return proj_elem
        else:
            return []

    def get_arc(self, name, proj_elem):
        """Get properties of a defined arc.

        Args:
            name (str): name of the arc
            params (object): object to fill with the projection element properties

        Returns:
            object: object with the properties of the polyline
        """
        arc = self.__thrift_client.GetCircleSegment(name)
        if arc:
            proj_elem.position.x = arc.center.x
            proj_elem.position.y = arc.center.y
            proj_elem.angle[0]   = arc.startAngle
            proj_elem.angle[1]   = arc.endAngle
            proj_elem.size[0]    = arc.radius
            return proj_elem
        else:
            return []

    def get_oval(self, name, proj_elem):
        """Get properties of a defined oval.

        Args:
            name (str): name of the oval
            params (object): object to fill with the projection element properties

        Returns:
            object: object with the properties of the polyline
        """
        oval = self.__thrift_client.GetOvalSegment(name)
        if oval:
            proj_elem.position.x = oval.center.x
            proj_elem.position.y = oval.center.y
            proj_elem.angle[0]   = oval.angle
            proj_elem.size[0]    = oval.width
            proj_elem.size[1]    = oval.height
            return proj_elem
        else:
            return []

    def get_text(self, name, proj_elem):
        """Get properties of a defined text.

        Args:
            name (str): name of the text
            params (object): object to fill with the projection element properties

        Returns:
            object: object with the properties of the polyline
        """
        text = self.__thrift_client.GetTextElement(name)
        if text:
            proj_elem.position.x = text.position.x
            proj_elem.position.y = text.position.y
            proj_elem.angle[0]   = text.angle
            proj_elem.size[0]    = text.height
            proj_elem.size[1]    = text.charSpacing
            proj_elem.text       = text.text
            return proj_elem
        else:
            return []

    def activate_figure(self, figure_params, status): 
        """Hide (deactivate) or unhide (activate hidden) a projection element from the active reference system.

        Args:
            figure_params (object): object with the necessary parameters to identify the projection element
            status (bool): true if activate projection element, false otherwise
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value is 
            an information message string
        """
        try:
            figure_type = figure_params.figure_type
            group       = figure_params.projection_group
            id          = figure_params.figure_name

            name = group + self.figures_list[figure_type] + id

            success = True
            if figure_type == Figure.POLYLINE:
                polyline = self.__thrift_client.GetPolyLine(name)
                if polyline:
                    polyline.activated = status
                    self.__thrift_client.SetPolyLine(polyline)
            elif figure_type == Figure.CIRCLE:
                circle = self.__thrift_client.GetCircleSegment(name)
                if circle:
                    circle.activated = status
                    self.__thrift_client.SetCircleSegment(circle)
            elif figure_type == Figure.ARC:
                arc = self.__thrift_client.GetCircleSegment(name)
                if arc:
                    arc.activated = status
                    self.__thrift_client.SetCircleSegment(arc)
            elif figure_type == Figure.OVAL:
                oval = self.__thrift_client.GetOvalSegment(name)
                if oval:
                    oval.activated = status
                    self.__thrift_client.SetOvalSegment(oval)
            elif figure_type == Figure.TEXT:
                text = self.__thrift_client.GetTextElement(name)
                if text:
                    text.activated = status
                    self.__thrift_client.SetTextElement(text)
            else:
                success = False
                message = "Figure " + name + "does not exist."

            if success and status:
                message = "Figure " + name + "reactivated."
            elif success and not status:
                message = "Figure " + name + "deactivated."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def delete_figure(self, figure_params):
        """Delete a projection element from the active reference system.

        Args:
            figure_params (object): object with the necessary parameters to identify the projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value is 
            an information message string
        """
        try:
            figure_type = figure_params.figure_type
            group       = figure_params.projection_group
            id          = figure_params.figure_name

            name = group + self.figures_list[figure_type] + id
            self.__thrift_client.RemoveGeoTreeElem(name)
            success = True
            message = "Figure removed"
        except Exception as e:
            success = False 
            message = e

        return success,message

    def translate_figure(self, figure_params, dx=0, dy=0, dz=0):
        """Translate a projection element from one position to another.

        Args:
            figure_params (object): object with the necessary parameters to identify the projection element
            dx (float): offset in x direction
            dy (float): offset in y direction
            dz (float): offset in z direction
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value
            is an information message string
        """
        try:
            figure_type = figure_params.figure_type
            group       = figure_params.projection_group
            id          = figure_params.figure_name

            name = group + self.figures_list[figure_type] + id

            self.__thrift_client.Translate(name,dx,dy)
            self.__thrift_client.ApplyTransformation(name)

            success = True
            message = "Figure translated"

        except Exception as e:
            success = False 
            message = e

        return success,message

    def scale_figure(self, figure_params, scale_factor):
        """Scale size of a projection element.

        Args:
            figure_params (object): object with the necessary parameters to identify the projection element
            scale_factor (float): scalation factor of the projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            figure_type = figure_params.figure_type
            group       = figure_params.projection_group
            id          = figure_params.figure_name

            name = group + self.figures_list[figure_type] + id

            self.__thrift_client.Scale(name,scale_factor)
            self.__thrift_client.ApplyTransformation(name)

            success = True
            message = "Figure scalated."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def rotate_figure(self, figure_params, x_angle, y_angle, z_angle):
        """Rotate a projection element an angle.

        Args:
            proj_elem_params (object): object with the necessary parameters to identify the projection element
            rotation_angle (float): rotation angle of the projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value is 
            an information message string
        """
        try:
            figure_type = figure_params.figure_type
            group       = figure_params.projection_group
            id          = figure_params.figure_name

            name = group + self.figures_list[figure_type] + id

            self.__thrift_client.Rotate(name,x_angle,y_angle,z_angle)
            self.__thrift_client.ApplyTransformation(name)

            success = True
            message = "Figure rotated."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def cs_axes_create(self, cs_params, proj_elem_params):
        """Create projection elements of the reference system's origin axes.

        Args:
            cs_params (object): object with the definition parameters of the reference system
            proj_elem_params (object): object to fill with the projection element properties

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value is 
            an information message string
        """

        proj_elem_params.projection_group = cs_params.name + "_origin"
        proj_elem_params.figure_name = "axis_x"
        proj_elem_params.position.x  = 0
        proj_elem_params.position.y  = 0
        proj_elem_params.size[0]     = GeometryTool.vector_point_distance(cs_params.P[1],cs_params.P[0])/2
        proj_elem_params.angle[0]    = 0
        
        success,message = self.define_arrow(cs_params.name,proj_elem_params)
        if not success:
            return success,message
        
        proj_elem_params.figure_name = "axis_y"
        proj_elem_params.angle[0]    = 90
        
        success,message = self.define_arrow(cs_params.name,proj_elem_params)
        if not success:
            return success,message

        success = True
        message = "Coordinate system origin axes created."
        return success,message

    def cs_frame_create(self, cs_name, proj_elem_params, points):
        """Create projection element of the reference system's frame.

        Args:
            cs_name (str): name of the reference system
            proj_elem_params (object): object to fill with the projection element properties
            points (list): list of the user system reference points

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value is 
            an information message string
        """
        proj_elem_params.projection_group = cs_name + "_origin"
        proj_elem_params.figure_name = "frame"

        success,message = self.define_rectangle(cs_name, proj_elem_params, points)
        if not success:
            return success,message

        success = True
        message = "Coordinate system frame created."
        return success,message