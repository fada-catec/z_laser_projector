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

"""This module uses ezdxf library to read dxf elements and cast them to figure type parameters, ready to be published on ros topic."""

import ezdxf
import math

from z_laser_zlp1.zlp_utils import ProjectionElementParameters, GeometryTool
from z_laser_msgs.msg import Figure
from geometry_msgs.msg import Point

def get_params(elem, file_name, shape_type, id):
    """Get params from elem on figure format to be published. It reads data depending on shape type

    Args:
        elem (modelspace elem): element from dxf containing shape data
        file_name (str): name of dxf file to be set as projection group
        shape_type (str): type of shape loaded from dxf
        id (int): number to identify shape, append to figure name

    Raises:
        TypeError: if shape type is not allowed

    Returns:
        ProjectionElementParameters: figure parameters
    """
    if shape_type == "LINE":
        params = get_dxf_line_params(elem, file_name, id)

    elif shape_type == "CIRCLE":
        params = get_dxf_circle_params(elem, file_name, id)

    elif shape_type == "ARC":
        params = get_dxf_arc_params(elem, file_name, id)

    elif shape_type == "ELLIPSE":
        params = get_dxf_ellipse_params(elem, file_name, id)

    elif shape_type == "TEXT":
        params = get_dxf_text_params(elem, file_name, id)

    else:
        raise TypeError("figure not identified: %s" % shape_type)

    return params

def get_dxf_line_params(elem, file_name, id):
    """Read params from elem and create LINE figure

    Args:
        elem (modelspace elem): element from dxf containing shape data
        file_name (str): name of dxf file to be set as projection group
        id (int): number to identify shape, append to figure name

    Returns:
        ProjectionElementParameters: figure parameters
    """
    proj_elem_params                  = ProjectionElementParameters()
    proj_elem_params.figure_type      = Figure.POLYLINE
    proj_elem_params.projection_group = file_name
    proj_elem_params.figure_name      = "line_" + str(id)
    proj_elem_params.position.x       = elem.dxf.start.x
    proj_elem_params.position.y       = elem.dxf.start.y
    proj_elem_params.size[0]          = GeometryTool.vector_point_distance(elem.dxf.start, elem.dxf.end)
    proj_elem_params.angle[0]         = GeometryTool.vector_point_angle(elem.dxf.start, elem.dxf.end)

    return proj_elem_params.to_figure()

def get_dxf_circle_params(elem, file_name, id):
    """Read params from elem and create CIRCLE figure

    Args:
        elem (modelspace elem): element from dxf containing shape data
        file_name (str): name of dxf file to be set as projection group
        id (int): number to identify shape, append to figure name

    Returns:
        ProjectionElementParameters: figure parameters
    """
    proj_elem_params                  = ProjectionElementParameters()
    proj_elem_params.figure_type      = Figure.CIRCLE
    proj_elem_params.projection_group = file_name
    proj_elem_params.figure_name      = "circle_" + str(id)
    proj_elem_params.position.x       = elem.dxf.center.x
    proj_elem_params.position.y       = elem.dxf.center.y
    proj_elem_params.size[0]          = elem.dxf.radius

    return proj_elem_params.to_figure()

def get_dxf_arc_params(elem, file_name, id):
    """Read params from elem and create ARC figure

    Args:
        elem (modelspace elem): element from dxf containing shape data
        file_name (str): name of dxf file to be set as projection group
        id (int): number to identify shape, append to figure name

    Returns:
        ProjectionElementParameters: figure parameters
    """
    proj_elem_params                  = ProjectionElementParameters()
    proj_elem_params.figure_type      = Figure.ARC
    proj_elem_params.projection_group = file_name
    proj_elem_params.figure_name      = "arc_" + str(id)
    proj_elem_params.position.x       = elem.dxf.center.x
    proj_elem_params.position.y       = elem.dxf.center.y
    proj_elem_params.size[0]          = elem.dxf.radius
    proj_elem_params.angle[0]         = elem.dxf.start_angle 
    proj_elem_params.angle[1]         = elem.dxf.end_angle

    return proj_elem_params.to_figure()

def get_dxf_ellipse_params(elem, file_name, id):
    """Read params from elem and create ELLIPSE figure

    Args:
        elem (modelspace elem): element from dxf containing shape data
        file_name (str): name of dxf file to be set as projection group
        id (int): number to identify shape, append to figure name

    Returns:
        ProjectionElementParameters: figure parameters
    """
    proj_elem_params                  = ProjectionElementParameters()
    proj_elem_params.figure_type      = Figure.OVAL
    proj_elem_params.projection_group = file_name
    proj_elem_params.figure_name      = "oval_" + str(id)
    proj_elem_params.position.x       = elem.dxf.center.x
    proj_elem_params.position.y       = elem.dxf.center.y
    proj_elem_params.size[0]          = GeometryTool.vector_point_distance(Point(0,0,0), elem.dxf.major_axis)
    proj_elem_params.size[1]          = elem.dxf.ratio * proj_elem_params.size[0]
    proj_elem_params.angle[0]         = GeometryTool.vector_point_angle(Point(0,0,0), elem.dxf.major_axis)

    return proj_elem_params.to_figure()

def get_dxf_text_params(elem, file_name, id):
    """Read params from elem and create TEXT figure

    Args:
        elem (modelspace elem): element from dxf containing shape data
        file_name (str): name of dxf file to be set as projection group
        id (int): number to identify shape, append to figure name

    Returns:
        ProjectionElementParameters: figure parameters
    """
    proj_elem_params                  = ProjectionElementParameters()
    proj_elem_params.figure_type      = Figure.TEXT
    proj_elem_params.projection_group = file_name
    proj_elem_params.figure_name      = "text_" + str(id)
    proj_elem_params.position.x       = elem.dxf.insert.x - elem.dxf.height*math.sin(math.radians(elem.dxf.rotation))
    proj_elem_params.position.y       = elem.dxf.insert.y + elem.dxf.height*math.cos(math.radians(elem.dxf.rotation))
    proj_elem_params.size[0]          = elem.dxf.height
    proj_elem_params.angle[0]         = elem.dxf.rotation
    proj_elem_params.text             = elem.dxf.text

    return proj_elem_params.to_figure()