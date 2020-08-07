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

"""This module imports the zlp and utils libraries and manages the functionalities provided from a layer of abstraction, simplifying the 
task of developing advanced applications."""

import sys
import time
import math
from z_laser_projector.zlp import ProjectorClient, CoordinateSystem, ProjectionElementControl
from z_laser_projector.utils import CoordinateSystemParameters, ProjectionElementParameters

class ProjectorManager:
    """This class uses the methods from the libraries imported."""

    def __init__(self, projector_IP = "192.168.10.10", server_IP = "192.168.10.11", connection_port = 9090, lic_path=""):
        """Initialize the ProjectorManager object."""
        self.projector_IP = projector_IP
        self.server_IP = server_IP
        self.connection_port = connection_port
        self.license_path = lic_path
        self.projector_id = ""
        self.coordinate_system = ""
        self.current_user_T_points = []

        self.projector_client = ProjectorClient() 

    def connect_and_setup(self):
        """Prepare projector to be used: connect, load license and activate.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        success,message = self.client_server_connect()
        if not success:
            return success,message

        success,message = self.load_license(self.license_path)
        if not success: 
            return success,message
        
        success,message = self.activate()
        if not success:
            return success,message

        success,message = self.geotree_operator_create()
        if not success:
            return success,message

        return True, "Projector ready"

    def client_server_connect(self):
        """Connect to thrift server of ZLP-Service.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        success,message = self.projector_client.connect(self.server_IP, self.connection_port)
        return success,message

    def client_server_disconnect(self):
        """Disconnect from thrift server of ZLP-Service.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        success,message = self.projector_client.disconnect()
        return success,message

    def load_license(self,license_path):
        """Transfer license file to service and check correct loading.

        Args:
            license_path (str): path of the license file

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        success,message = self.projector_client.transfer_license(license_path)
        return success,message
    
    def activate(self):
        """Activate projector.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        self.projector_id,success,message = self.projector_client.activate_projector(self.projector_IP)
        if success:
            success,message = self.projector_client.check_license()
            if success:
                message = "Projector properly activated."
        return success,message

    def deactivate(self):
        """Deactivate projector.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        success,message = self.projector_client.deactivate_projector()
        return success,message

    def geotree_operator_create(self):
        """Create geotree operator to handle reference systems and projection figures.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        module_id,success,message = self.projector_client.function_module_create()
        thrift_client = self.projector_client.get_thrift_client()

        self.cs_element = CoordinateSystem(self.projector_id, module_id, thrift_client)
        self.projection_element = ProjectionElementControl(module_id,thrift_client)

        if not self.coordinate_system:
            message = message + "\nNo Current Coordinate System set so far."

        return success,message

    def start_projection(self):
        """Start projection of figures associated to the current reference system.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        success,message = self.projector_client.start_project(self.coordinate_system)
        return success,message 

    def stop_projection(self):
        """Stop projection of all figures.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        success,message = self.projector_client.stop_project()
        return success,message

    def get_coordinate_systems(self):
        """Get list of all defined reference systems.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        cs_list,success,message = self.cs_element.coordinate_system_list()
        return cs_list,success,message

    def define_coordinate_system(self,cs_params):
        """Define a new coordinate reference system.

        Args:
            cs_params (list): list of necessary parameters to define a new reference system (coordinates of reference system points)

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        coord_sys,self.current_user_T_points,success,message = self.cs_element.define_cs(cs_params)
        if success:
            success,message = self.register_coordinate_system(coord_sys)
            if success:
                success,message = self.set_coordinate_system(coord_sys)
                message = "Coordinate system defined, registered and set."

        return success,message

    def register_coordinate_system(self,coord_sys):
        """Register new coordinate reference system.

        Args:
            cs_params (list): list of parameters from the new defined reference system

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        success,message = self.cs_element.register_cs(coord_sys)
        return success,message

    def set_coordinate_system(self,coord_sys):
        """Set the current operating reference system.

        Args:
            coord_sys (str): name of reference system

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        success,message = self.cs_element.set_cs(coord_sys)
        if success:
            self.coordinate_system = coord_sys
        return success,message

    def show_coordinate_system(self,secs):
        """Project the reference points and origin axes of the current reference system on the surface.

        Args:
            secs (int): number of projection seconds on the surface 

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        if not self.coordinate_system:
            success = False
            message = "Coordinate system cannot be showed because there is none or the current is not set first."
            message = message + "\n NOTE: Check if the coordinate system is set first."
            return success,message
        
        success,message = self.cs_element.show_cs(self.coordinate_system, secs)
        return success,message

    def remove_coordinate_system(self,coord_sys):
        """Delete current reference system

        Args:
            coord_sys (str): name of reference system 

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        success,message = self.cs_element.remove_cs(coord_sys)
        if success:
            self.coordinate_system = ""
            self.current_user_T_points = []
            message = "Coordinate system removed. Define or set new one."

        return success,message

    def create_polyline(self,proj_elem_params):
        """Create a line as new projection figure, associated to the current reference system.

        Args:
            proj_elem_params (list): list of necessary parameters to define a new line 

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        if not self.coordinate_system:
            success = False
            message = "There is not a current coordinate system. Define or set one first."
            return success,message

        success,message = self.projection_element.define_polyline(self.coordinate_system,proj_elem_params)
        return success,message

    def hide_shape(self, proj_elem_params):
        """Hide a figure from current reference system.

        Args:
            proj_elem_params (list): list of necessary parameters to identify the figure 

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        success,message = self.projection_element.deactivate_shape(proj_elem_params)
        return success,message

    def unhide_shape(self,proj_elem_params):
        """Unhide a figure from current reference system.

        Args:
            proj_elem_params (list): list of necessary parameters to identify the figure 

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        success,message = self.projection_element.reactivate_shape(proj_elem_params)
        return success,message

    def remove_shape(self,proj_elem_params):
        """Delete a figure from current reference system.

        Args:
            proj_elem_params (list): list of necessary parameters to identify the figure  

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        success,message = self.projection_element.delete_shape(proj_elem_params)
        return success,message

    def cs_axes_create(self,cs_params):
        """Create projection figures of user reference system origin axes, project them and hide after.

        Args:
            cs_params (list): list of definition parameters of the reference system

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.shape_type            = "polyline"
        proj_elem_params.group_name = self.coordinate_system + "_origin"
        proj_elem_params.x                     = 0
        proj_elem_params.y                     = 0
        proj_elem_params.length                = cs_params.resolution/2
        
        proj_elem_params.shape_id     = "axis_x"
        proj_elem_params.angle        = 0
        success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params) 
        if success:
            success,message = self.hide_shape(proj_elem_params)
        
        if success:
            proj_elem_params.shape_id = "axis_y"
            proj_elem_params.angle    = 90
            success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params)
            if success:
                success,message = self.hide_shape(proj_elem_params)
        
        if success:
            proj_elem_params.shape_id = "axis_x_arrow1"
            proj_elem_params.x        = cs_params.resolution/2
            proj_elem_params.y        = 0
            proj_elem_params.length   = cs_params.resolution/12
            proj_elem_params.angle    = 180 - 15
            success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params)
            if success:
                success,message = self.hide_shape(proj_elem_params)
        
        if success:
            proj_elem_params.shape_id = "axis_x_arrow2"
            proj_elem_params.angle    = 180 + 15
            success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params)
            if success:
                success,message = self.hide_shape(proj_elem_params)
        
        if success:
            proj_elem_params.shape_id = "axis_y_arrow1"
            proj_elem_params.x        = 0
            proj_elem_params.y        = cs_params.resolution/2
            proj_elem_params.length   = cs_params.resolution/14
            proj_elem_params.angle    = 270 - 15
            success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params)
            if success:
                success,message = self.hide_shape(proj_elem_params)
        
        if success:
            proj_elem_params.shape_id = "axis_y_arrow2"
            proj_elem_params.angle    = 270 + 15
            success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params)
            if success:
                success,message = self.hide_shape(proj_elem_params)
        
        if success:
            message = "Coordinate system origin axes created correctly"

        return success,message

    def cs_axes_unhide(self):
        """Unhide user reference system origin axes, project them and hide after.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.group_name = self.coordinate_system + "_origin"
        proj_elem_params.shape_type = "polyline"
        
        proj_elem_params.shape_id     = "axis_x"
        success,message = self.unhide_shape(proj_elem_params)
        
        if success:
            proj_elem_params.shape_id = "axis_y"
            success,message = self.unhide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "axis_x_arrow1"
            success,message = self.unhide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "axis_x_arrow2"
            success,message = self.unhide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "axis_y_arrow1"
            success,message = self.unhide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "axis_y_arrow2"
            success,message = self.unhide_shape(proj_elem_params)
        
        if success:
            message = "Coordinate system origin axes unhided"

        return success,message

    def cs_axes_hide(self):
        """Hide user reference system origin axes, project them and hide after.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.group_name = self.coordinate_system + "_origin"
        proj_elem_params.shape_type = "polyline"
        
        proj_elem_params.shape_id     = "axis_x"        
        success,message = self.hide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "axis_y"
            success,message = self.hide_shape(proj_elem_params)

        if success: 
            proj_elem_params.shape_id = "axis_x_arrow1"
            success,message = self.hide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "axis_x_arrow2"
            success,message = self.hide_shape(proj_elem_params)
        
        if success:
            proj_elem_params.shape_id = "axis_y_arrow1"
            success,message = self.hide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "axis_y_arrow2"
            success,message = self.hide_shape(proj_elem_params)
        
        if success:
            message = "Coordinate system origin axes hided"

        return success,message

    def cs_frame_create(self,cs_params):
        """Create frame lines projection figures of user reference system, project them and hide them after.

        Args:
            T (list): list of the User System Reference Points
            cs_params (list): list of definition parameters of the reference system

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.shape_type            = "polyline"
        proj_elem_params.group_name = self.coordinate_system + "_frame"

        T = self.current_user_T_points
        proj_elem_params.shape_id     = "T1_T2"
        proj_elem_params.x            = T[0]
        proj_elem_params.y            = T[1]
        proj_elem_params.length       = math.sqrt((T[2]-T[0])**2+(T[3]-T[1])**2)
        proj_elem_params.angle        = 180/math.pi*math.atan2((T[3]-T[1]),(T[2]-T[0]))
        success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params) 
        if success:
            success,message = self.hide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "T2_T3"
            proj_elem_params.x        = T[2]
            proj_elem_params.y        = T[3]
            proj_elem_params.length   = math.sqrt((T[4]-T[2])**2+(T[5]-T[3])**2)
            proj_elem_params.angle    = 180/math.pi*math.atan2((T[5]-T[3]),(T[4]-T[2]))
            success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params)
            if success:
                success,message = self.hide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "T3_T4"
            proj_elem_params.x        = T[4]
            proj_elem_params.y        = T[5]
            proj_elem_params.length   = math.sqrt((T[6]-T[4])**2+(T[7]-T[5])**2)
            proj_elem_params.angle    = 180/math.pi*math.atan2((T[7]-T[5]),(T[6]-T[4]))
            success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params)
            if success:
                success,message = self.hide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "T4_T1"
            proj_elem_params.x        = T[6]
            proj_elem_params.y        = T[7]
            proj_elem_params.length   = math.sqrt((T[0]-T[6])**2+(T[1]-T[7])**2)
            proj_elem_params.angle    = 180/math.pi*math.atan2((T[1]-T[7]),(T[0]-T[6]))
            success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params)
            if success:
                success,message = self.hide_shape(proj_elem_params)

        if success:
            message = "User System frame's lines unhide"

        return success,message

    def cs_frame_unhide(self):
        """Unhide frame lines of user reference system, project them and hide after.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.group_name = self.coordinate_system + "_frame"
        proj_elem_params.shape_type = "polyline"
        
        proj_elem_params.shape_id     = "T1_T2"
        success,message = self.unhide_shape(proj_elem_params)
        
        if success:
            proj_elem_params.shape_id = "T2_T3"
            success,message = self.unhide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "T3_T4"
            success,message = self.unhide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "T4_T1"
            success,message = self.unhide_shape(proj_elem_params)
        
        return success,message


    def cs_frame_hide(self):
        """Hide frame lines of user reference system, project them and hide after.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.group_name = self.coordinate_system + "_frame"
        proj_elem_params.shape_type = "polyline"
        
        proj_elem_params.shape_id     = "T1_T2"
        success,message = self.hide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "T1_T2"
            success,message = self.hide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "T2_T3"
            success,message = self.hide_shape(proj_elem_params)

        if success:
            proj_elem_params.shape_id = "T3_T4"
            success,message = self.hide_shape(proj_elem_params)
        
        if success:
            message = "User System frame's lines hide"

        return success,message