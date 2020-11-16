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

from scipy.spatial import distance
import math

from geometry_msgs.msg import Point
from z_laser_msgs.msg import Figure
from z_laser_msgs.srv import CoordinateSystemRequest


class UserT(object):
    def __init__(self, x0, y0, resolution, size_horiz, size_vert):
        T0 = Point()
        T1 = Point()
        T2 = Point()
        T3 = Point()

        T0.x = x0
        T0.y = y0
        T1.x = T0.x + resolution*size_horiz/max(size_horiz,size_vert)
        T1.y = T0.y
        T2.x = T1.x
        T2.y = T0.y + resolution*size_vert/max(size_horiz,size_vert)
        T3.x = T0.x
        T3.y = T2.y

        self.T = [T0, T1, T2, T3]

class UserP(object):
    def __init__(self):
        P0 = Point()
        P1 = Point()
        P2 = Point()
        P3 = Point()

        self.P = [P0, P1, P2, P3]

class CoordinateSystemParameters(object):
    """This class is used as data object with the necessary parameters to define a coordinate system.

    Attributes:
        name (str): coordinate system name
        distance (float): distance between the projection surface and the projector device
        resolution (float): resolution of user reference system {T}
        P1 (object): object with the x,y,z position of point p1 from projector reference system {P}
        P2 (object): object with the x,y,z position of point p2 from projector reference system {P}
        P3 (object): object with the x,y,z position of point p3 from projector reference system {P}
        P4 (object): object with the x,y,z position of point p4 from projector reference system {P}
        T1 (object): object with the x,y,z position of point T1 from user reference system {T}
        T2 (object): object with the x,y,z position of point T2 from user reference system {T}
        T3 (object): object with the x,y,z position of point T3 from user reference system {T}
        T4 (object): object with the x,y,z position of point T4 from user reference system {T}
    """
    DEFAULT_SHOW_TIME = 1

    def __init__(self):
        """Initialize the CoordinateSystemParameters object."""
        self.name        = str()
        self.distance    = float()
        self.resolution  = float()
        self.P           = [Point(), Point(), Point(), Point()]
        self.T           = [Point(), Point(), Point(), Point()]

    @staticmethod
    def req_to_param(req):
        params = CoordinateSystemParameters()
        params.name       = req.name
        params.distance   = req.distance
        params.resolution = req.resolution
        params.P = req.P
        params.T[0] = req.T0
        return params

    @staticmethod
    def param_to_req(params):
        req = CoordinateSystemRequest()
        req.name       = params.name
        req.distance   = params.distance
        req.resolution = params.resolution
        req.P = params.P
        req.T0 = params.T[0]
        return req


class ProjectionElementParameters(object):
    """This class is used as data object with the necessary information to create a projection element.
    
    Attributes:
        projection_group (str): name of the projection group to which the projection element belongs
        figure_name (str): name of the projection element to define
        curve_type (str): type of curve (circle, oval, arc)
        x (float): x-position of the projection element's 'characteristic point'
        The 'characteristic point' for a polyline refers to its starting point. For circle refers to its center 
        y (float): y-position of the projection element's 'characteristic point'
        angle (float): angle property for different figures (polyline slope, arc starting angle, text tilt, ...)
        end_angle (float): end angle of arc figure
        length (float): length property for different figures (polyline length, circle radius, text string wide, 
        oval length size ...)
        height (float): height property for different figures (text characters height, oval width size, ...)
        text (str): text character string for text projection element
        char_spacing (float): space between characters
        ..........................CAMBIAR.............................................
    """
    figures_list = ["/polyline/",
                    "/circle/",
                    "/arc/",
                    "/oval/",
                    "/text/"]

    def __init__(self):
        """Initialize the ProjectionElementParameters object."""
        self.figure_type      = int()
        self.projection_group = str()
        self.figure_name      = str()
        self.position         = Point()
        self.size             = [float(),float()]
        self.angle            = [float(),float()]
        self.text             = str()

    def to_figure(self):
        figure = Figure()
        figure.projection_group = self.projection_group
        figure.figure_type      = self.figure_type
        figure.figure_name      = self.figure_name
        figure.position         = self.position
        figure.size             = self.size
        figure.angle            = self.angle
        figure.text             = self.text
        return figure
    

class GeometryTool(object):
    """This class implement functions to generate basic geometry and math tools.
    
    Args:
        thrift_client (object): object with the generated client to communicate with the projector
    """
    def __init__(self, thrift_client):
        """Initialize the GeometryTool object."""
        self.__thrift_client = thrift_client

    def create_matrix4x4(self):
        """Initialize 4x4 matrix.
            
        Returns:
            object: matrix struct initialized with empty values
        """
        mat = self.__thrift_client.thrift_interface.Matrix4x4(list())
        return mat

    def create_2d_point(self, x=0, y=0):
        """Initialize 2-dimension array.
            
        Args:
            x (float): x position value
            y (float): y position value

        Returns:
            object: struct with the values of the 2 axes (x,y)
        """
        return self.__thrift_client.thrift_interface.Vector2D(x, y)

    def create_3d_point(self, x=0, y=0, z=0):
        """Initialize 3-dimension array.
            
        Args:
            x (float): x position value
            y (float): y position value
            z (float): z position value

        Returns:
            object: struct with the values of the 3 axes (x,y,z)
        """
        return self.__thrift_client.thrift_interface.Vector3D(x, y, z)

    @staticmethod
    def vector_point_distance(point1, point2):
        return distance.euclidean((point1.x, point1.y), (point2.x, point2.y))

    @staticmethod
    def vector_point_angle(point1, point2):
        return 180/math.pi*math.atan2((point2.y-point1.y), (point2.x-point1.x))
