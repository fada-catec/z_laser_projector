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

"""Complementary module with useful classes to support the usage of zlp libraries."""

from scipy.spatial import distance
import math

from geometry_msgs.msg import Point
from z_laser_msgs.msg import Figure
from z_laser_msgs.srv import CoordinateSystemRequest


class UserT(object):
    """This class provides a method to calculate the user points T1, T2, T3 from T0 and 
    the properties of the reference system.
    
    Args:
        x0 (float): x-position of the user T0 point
        y0 (float): y-position of the user T0 point
        resolution (float): resolution of the user reference system {T}
        size_horiz (float): horizontal dimension of the refence system in real dimensions {P}
        size_vert (float): vertical size of the refence system in real dimensions {P}

    Attributes:
        T (list): list of the calculated user {T} points
    """
    def __init__(self, x0, y0, resolution, size_horiz, size_vert):
        """Initialize the UserT object."""
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

class CoordinateSystemParameters(object):
    """This class is used as data object with the necessary parameters to define a coordinate system.

    Attributes:
        DEFAULT_SHOW_TIME (int): number of seconds for projecting reference system's projection elements
        name (str): reference system name
        distance (float): distance between the projection surface and the projector device
        resolution (float): resolution of user reference system {T}
        P (list): list with the x,y,z position of reference points from projector reference system {P}
        T (list): list with the x,y,z position of user reference points from user reference system {T}
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
        """Convert ROS request object to CoordinateSystemParameters object.

        Args:
            req (object): ROS request object with the parameters of a coordinate system

        Returns:
            object: same parameters of the coordinate system now in a CoordinateSystemParameters object 
        """
        params = CoordinateSystemParameters()
        params.name       = req.name
        params.distance   = req.distance
        params.resolution = req.resolution
        params.P = req.P
        params.T[0] = req.T0
        return params

    @staticmethod
    def param_to_req(params):
        """Convert CoordinateSystemParameters object to ROS request object.

        Args:
            params (object): CoordinateSystemParameters object with the parameters of a coordinate system

        Returns:
            object: same parameters of the coordinate system now in a ROS request object
        """
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
        figures_list (list): list with the projection elements' identificator names
        figure_type (int): projection element id (see Figure.msg for more information)
        projection_group (str): name of the projection group to which the projection element belongs
        figure_name (str): name of the projection element to define
        position (object): object with the x,y,z position of the projection element's characteristic point
        size (list): list with the characteristic sizes of the projection element
        angle (list): list with the characteristic angles of the projection element
        text (str): text character string for text projection element
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
        """Return ProjectionElementParameters attributes converted to ROS msg format.

        Returns:
            object: ROS msg object with the parameters of the projection element
        """
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
            object: matrix object initialized with empty values
        """
        mat = self.__thrift_client.thrift_interface.Matrix4x4(list())
        return mat

    def create_2d_point(self, x=0, y=0):
        """Initialize 2-dimension array.
            
        Args:
            x (float): x position value
            y (float): y position value

        Returns:
            object: object with the values of the 2 axes (x,y)
        """
        return self.__thrift_client.thrift_interface.Vector2D(x, y)

    def create_3d_point(self, x=0, y=0, z=0):
        """Initialize 3-dimension array.
            
        Args:
            x (float): x position value
            y (float): y position value
            z (float): z position value

        Returns:
            object: object with the values of the 3 axes (x,y,z)
        """
        return self.__thrift_client.thrift_interface.Vector3D(x, y, z)

    @staticmethod
    def vector_point_distance(point1, point2):
        """Return the euclidean distance between 2 points.

        Args:
            point1 (list): x,y,z position of the first point
            point2 (list): x,y,z position of the second point

        Returns:
            float: euclidean distance between point1 and point2
        """
        return distance.euclidean((point1.x, point1.y), (point2.x, point2.y))

    @staticmethod
    def vector_point_angle(point1, point2):
        """Angle of the vector consisting in two points, regards to the horizontal 0 degrees.

        Args:
            point1 (list): x,y,z position of the first point of the vector
            point2 (list): x,y,z position of the second point of the vector

        Returns:
            float: vector angle regards to the horizontal 0 degrees
        """
        return 180/math.pi*math.atan2((point2.y-point1.y), (point2.x-point1.x))
