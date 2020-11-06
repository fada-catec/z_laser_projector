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

class Point3D(object):
    def __init__(self,x=0,y=0,z=0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

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
    DEFAULT_WAIT_TIME = 1

    def __init__(self):
        """Initialize the CoordinateSystemParameters object."""
        self.name        = str()
        self.distance    = float()
        self.resolution  = float()
        self.P1          = Point3D()
        self.P2          = Point3D()
        self.P3          = Point3D()
        self.P4          = Point3D()
        self.T1          = Point3D()
        self.T2          = Point3D()
        self.T3          = Point3D()
        self.T4          = Point3D()

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
        self.position         = Point3D()
        self.size             = [float(),float()]
        self.angle            = [float(),float()]
        self.text             = str()

class KeyboardParameters(object):
    """."""

    def __init__(self):
        """."""
        from pynput import keyboard

        self.KEY_UP    = {keyboard.Key.up}
        self.KEY_DOWN  = {keyboard.Key.down}
        self.KEY_LEFT  = {keyboard.Key.left}
        self.KEY_RIGHT = {keyboard.Key.right}
        self.KEY_PLUS  = {keyboard.KeyCode(char='+')}
        self.KEY_MINUS = {keyboard.KeyCode(char='-')}
        self.COMB_1    = {keyboard.Key.left, keyboard.Key.ctrl}
        self.COMB_2    = {keyboard.Key.right, keyboard.Key.ctrl}
        self.ESC       = {keyboard.Key.esc}

        self.COMBINATIONS = [self.KEY_UP,self.KEY_UP,self.KEY_DOWN,self.KEY_LEFT,self.KEY_RIGHT,
                            self.KEY_PLUS,self.KEY_MINUS,self.COMB_1,self.COMB_2,self.ESC]