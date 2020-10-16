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

"""This module imports the core and utils libraries and manages the functionalities provided from a layer of abstraction, simplifying the 
task of developing advanced applications."""

import sys
import math

from z_laser_zlp1.zlp_core import ProjectorClient, CoordinateSystem, ProjectionElementControl, KeyboardControl
from z_laser_zlp1.zlp_utils import CoordinateSystemParameters, ProjectionElementParameters

class ZLPProjectorManager(object):
    """This class envolves the methods from the libraries imported.
    
    Args:
        projector_IP (str): IP number of projector device
        server_IP (str): IP number of service running at projector device
        connection_port (int): connection port number 

    Attributes:
        projector_client (object): ProjectorClient object from utils library
    """
    def __init__(self, projector_IP = "192.168.10.10", server_IP = "192.168.10.11", connection_port = 9090, lic_path=""):
        """Initialize the ZLPProjectorManager object."""
        self.__projector_IP = projector_IP
        self.__server_IP = server_IP
        self.__connection_port = connection_port
        self.__license_path = lic_path
        self.__projector_id = ""
        self.__coordinate_system = ""

        self.projector_client = ProjectorClient() 

    def connect_and_setup(self):
        """Prepare projector to be used: connect, load license and activate.
        
        Raises:
            SystemError
        """
        try:
            self.client_server_connect()       
            self.load_license(self.__license_path)
            self.activate()
            self.geotree_operator_create()
        
        except Exception as e:
            raise SystemError(e)

    def client_server_connect(self):
        """Connect to thrift server of ZLP-Service.

        Raises:
            ConnectionError
        """
        success,message = self.projector_client.connect(self.__server_IP, self.__connection_port)
        if not success:
            raise ConnectionError(message)

    def client_server_disconnect(self):
        """Disconnect from thrift server of ZLP-Service.

        Raises:
            ConnectionError
        """
        success,message = self.projector_client.disconnect()
        if success:
            self.__coordinate_system = ""
        else:
            raise ConnectionError(message)

    def get_connection_status(self):
        """Get connection status of projector device.

        Returns:
            status(bool):
        """
        status = self.projector_client.connection_status()
        return status

    def load_license(self,license_path):
        """Transfer license file to service.

        Args:
            license_path (str): path of the license file

        Raises:
            FileNotFoundError
        """
        success,message = self.projector_client.transfer_license(license_path)
        if not success:
            raise FileNotFoundError(message)

    def activate(self):
        """Activate projector once it is connected and license is transfered.

        Raises:
            SystemError
        """
        self.__projector_id,success,message = self.projector_client.activate_projector(self.__projector_IP)
        if not success:
            raise SystemError(message)
    
        success,message = self.projector_client.check_license()
        if not success:
            raise SystemError(message)

    def deactivate(self):
        """Deactivate projector.

        Raises:
            SystemError
        """
        success,message = self.projector_client.deactivate_projector()
        if not success:
            raise SystemError(message)

    def geotree_operator_create(self):
        """Create geotree operator to handle reference systems and projection elements.

        Raises:
            SystemError
        """
        module_id,success,message = self.projector_client.function_module_create()
        if not success:
            raise SystemError(message)

        thrift_client = self.projector_client.get_thrift_client()

        self.cs_element = CoordinateSystem(self.__projector_id, module_id, thrift_client)
        self.projection_element = ProjectionElementControl(module_id,thrift_client)
        self.keyboard_control = KeyboardControl(self.projector_client,self.projection_element)

    def start_projection(self):
        """Start projection of elements associated to the active reference system.

        Raises:
            Warning
            SystemError
        """
        if not self.__coordinate_system:
            raise Warning("No Active Coordinate System set yet.")

        success,message = self.projector_client.start_project(self.__coordinate_system)
        if not success:
            raise SystemError(message)

    def stop_projection(self):
        """Stop projection of all elements.

        Raises:
            SystemError
        """
        success,message = self.projector_client.stop_project()
        if not success:
            raise SystemError(message)

    def get_coordinate_systems_list(self):
        """Get list of all defined reference systems.

        Raises:
            SystemError

        Return:
            cs_list(list): list of defined coordinate systems
            (string): name of the active coordinate system if it exists
        """
        cs_list,success,message = self.cs_element.coordinate_system_list()
        if not success:
            raise SystemError(message)

        return cs_list,self.__coordinate_system

    def get_coordinate_system_params(self,coord_sys):
        """Get parameters values of a defined coordinate system.

        Args:
            coord_sys (str): name of reference coordinate system 

        Raises:
            SystemError
        """
        cs_params = CoordinateSystemParameters()
        cs_params,success,message = self.cs_element.get_cs(coord_sys,cs_params)
        if not success:
            raise SystemError(message)

        return cs_params

    def define_coordinate_system(self,cs_params,do_target_search):
        """Define a new coordinate reference system.

        Args:
            cs_params (list): list of necessary parameters to define a new reference system (coordinates of reference system points)

        Raises:
            SystemError
        """
        success,message,self.cs_scanned = self.cs_element.define_cs(cs_params,do_target_search)
        if not success:
            raise SystemError(message)
        elif success and do_target_search:
            success,message,_ = self.cs_element.define_cs(self.cs_scanned,False)
        
        try:
            self.register_coordinate_system(cs_params.name)
            self.set_coordinate_system(cs_params.name)
        except SystemError as e:
            raise SystemError(e)

    def register_coordinate_system(self,coord_sys):
        """Register new coordinate reference system.

        Args:
            cs_params (list): list of parameters from the new defined reference system

        Raises:
            SystemError
        """
        success,message = self.cs_element.register_cs(coord_sys)
        if not success:
            raise SystemError(message)
        
    def set_coordinate_system(self,coord_sys):
        """Set the active reference system.

        Args:
            coord_sys (str): name of reference coordinate system

        Raises:
            SystemError
        """
        success,message = self.cs_element.set_cs(coord_sys)
        if success:
            self.__coordinate_system = coord_sys
        if not success:
            raise SystemError(message)
        
    def show_coordinate_system(self):
        """Project the reference points of the active reference system on the projection surface.

        Args:
            secs (int): number of projection seconds on the surface 

        Raises:
            SystemError
        """
        if not self.__coordinate_system:
            message = "No active coordinate system set yet."
            raise SystemError(message)
        
        print("active coordinate system {}".format(self.__coordinate_system))
        success,message = self.cs_element.show_cs(self.__coordinate_system)
        if not success:
            raise SystemError(message)

    def hide_coordinate_system(self):
        """Project the reference points, origin axes and frame of the active reference system, 
        on the projection surface.

        Args:
            secs (int): number of projection seconds on the surface 

        Raises:
            SystemError
        """
        if not self.__coordinate_system:
            message = "Coordinate system does not exist."
            raise SystemError(message)
        
        success,message = self.cs_element.hide_cs(self.__coordinate_system)
        if not success:
            raise SystemError(message)

    def show_frame(self):
        """Project the origin axes and frame of the active reference system on the projection surface.

        Raises:
            SystemError
        """
        if not self.__coordinate_system:
            message = "Coordinate system does not exist."
            raise SystemError(message)
        try:
            self.cs_frame_unhide()
            self.cs_axes_unhide()
            self.start_projection()

        except SystemError as e:
            raise SystemError(e) 
        
    def hide_frame(self):
        """Hide the origin axes and frame of the active reference system.

        Raises:
            SystemError
        """
        if not self.__coordinate_system:
            message = "Coordinate system does not exist."
            raise SystemError(message)
        try:
            self.stop_projection()
            self.cs_frame_hide()  
            self.cs_axes_hide()
        except SystemError as e:
            raise SystemError(e)  

    def remove_coordinate_system(self,coord_sys):
        """Delete a reference system.

        Args:
            coord_sys (str): name of reference coordinate system 

        Returns:
            (bool): 

        Raises:
            SystemError
        """
        success,message = self.cs_element.remove_cs(coord_sys)
        if success:
            if coord_sys == self.__coordinate_system:
                self.__coordinate_system = ""
                return True
        else:
            raise SystemError(message)
        
    def create_polyline(self,proj_elem_params):
        """Create a polyline as new projection element, associated to the active reference system.

        Args:
            proj_elem_params (object): object with the necessary parameters to define a new polyline 

        Raises:
            SystemError
        """
        if not self.__coordinate_system:
            message = "There is not an active coordinate system. Define or set one first."
            raise SystemError(message)
        
        success,message = self.projection_element.define_polyline(self.__coordinate_system,proj_elem_params)
        if not success:
            raise SystemError(message)

    def create_curve(self,proj_elem_params):
        """Create a curve (circle, oval, arc) as new projection element, associated to the active reference system.

        Args:
            proj_elem_params (object): object with the necessary parameters to define a new curve

        Raises:
            SystemError
        """
        if not self.__coordinate_system:
            message = "There is not an active coordinate system. Define or set one first."
            raise SystemError(message)

        if proj_elem_params.curve_type == "circle":
            success,message = self.projection_element.define_circle(self.__coordinate_system,proj_elem_params)
        elif proj_elem_params.curve_type == "arc":
            success,message = self.projection_element.define_arc(self.__coordinate_system,proj_elem_params)
        elif proj_elem_params.curve_type == "oval":
            success,message = self.projection_element.define_oval(self.__coordinate_system,proj_elem_params)
        else:
            message = "curve_type does not correspond to any possible figure"
            raise SystemError(message)
        
        if not success:
            raise SystemError(message)

    def create_text(self,proj_elem_params):
        """Create a text as new projection element, associated to the active reference system.

        Args:
            proj_elem_params (object): object with the necessary parameters to define a new text

        Raises:
            SystemError
        """
        if not self.__coordinate_system:
            message = "There is not an active coordinate system. Define or set one first."
            raise SystemError(message)

        success,message = self.projection_element.define_text(self.__coordinate_system,proj_elem_params)
        if not success:
            raise SystemError(message)
        
    def hide_proj_elem(self, proj_elem_params):
        """Hide a projection element from active reference system.

        Args:
            proj_elem_params (object): object with necessary parameters to identify a projection element

        Raises:
            SystemError
        """
        success,message = self.projection_element.activate_figure(proj_elem_params,False)
        if not success:
            raise SystemError(message)
        
    def unhide_proj_elem(self,proj_elem_params):
        """Unhide a projection element from active reference system.

        Args:
            proj_elem_params (object): object with necessary parameters to identify a projection element 

        Raises:
            SystemError
        """
        success,message = self.projection_element.activate_figure(proj_elem_params,True)
        if not success:
            raise SystemError(message)
        
    def remove_proj_elem(self,proj_elem_params):
        """Delete a projection element from active reference system.

        Args:
            proj_elem_params (object): object with necessary parameters to identify a projection element  

        Raises:
            SystemError
        """
        success,message = self.projection_element.delete_figure(proj_elem_params)
        if not success:
            raise SystemError(message)
    
    def monitor_proj_elem(self,proj_elem_params):
        """Monitor transformation operations (translation, rotation, scalation) to a specific figure on real time projection 
        by the use of keyboard.

        Args:
            proj_elem_params (list): list of necessary parameters to identify the figure  

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        success,message = self.projector_client.start_project(self.__coordinate_system)
        if success:
            success,message = self.keyboard_control.init_keyboard_listener(self.__coordinate_system,proj_elem_params)
            if not success:
                raise SystemError(message)
    
    def cs_axes_create(self,cs_params):
        """Create projection elements of user reference system origin axes.

        Args:
            cs_params (object): object with necessary parameters to identify a coordinate system

        Raises:
            SystemError
        """
        proj_elem_params = ProjectionElementParameters()
        success,message = self.projection_element.cs_axes_create(cs_params,proj_elem_params)
        if not success:
            raise SystemError(message)

        try:
            self.cs_axes_hide()
        except SystemError as e:
            raise SystemError(e)
    
    def cs_frame_create(self,cs_params):
        """Create projection elements of user reference system frame.

        Args:
            T (list): list of the User System Reference Points
            cs_params (list): list of definition parameters of the reference system

        Raises:
            SystemError
        """
        proj_elem_params = ProjectionElementParameters()
        cs = self.get_coordinate_system_params(cs_params.name)
        T = [cs.T1.x, cs.T1.y, cs.T2.x, cs.T2.y, cs.T3.x, cs.T3.y, cs.T4.x, cs.T4.y]
        success,message = self.projection_element.cs_frame_create(cs_params,proj_elem_params,T)
        if not success:
            raise SystemError(message)

        try:
           self.cs_frame_hide()
        except SystemError as e:
            raise SystemError(e)

    def cs_axes_unhide(self):
        """Unhide user reference system origin axes.

        Raises:
            SystemError
        """
        proj_elem_params                  = ProjectionElementParameters()
        proj_elem_params.projection_group = self.__coordinate_system + "_origin"
        proj_elem_params.figure_type      = "polyline"
        
        try:
            for axis_id in self.projection_element.axes_ids:
                proj_elem_params.figure_name = axis_id
                self.unhide_proj_elem(proj_elem_params)            
            
        except SystemError as e:
            raise SystemError(e)
        
    def cs_axes_hide(self):
        """Hide user reference system origin axes.

        Raises:
            SystemError
        """
        proj_elem_params                  = ProjectionElementParameters()
        proj_elem_params.projection_group = self.__coordinate_system + "_origin"
        proj_elem_params.figure_type      = "polyline"
        
        try:
            for axis_id in self.projection_element.axes_ids:
                proj_elem_params.figure_name = axis_id
                self.hide_proj_elem(proj_elem_params)            

        except SystemError as e:
            raise SystemError(e)
        
    def cs_frame_unhide(self):
        """Unhide frame of user reference system.

        Raises:
            SystemError
        """        
        proj_elem_params                  = ProjectionElementParameters()
        proj_elem_params.projection_group = self.__coordinate_system + "_frame"
        proj_elem_params.figure_type      = "polyline"
                
        try:
            for frame_id in self.projection_element.frame_ids:
                proj_elem_params.figure_name = frame_id
                self.unhide_proj_elem(proj_elem_params)
                
        except SystemError as e:
            raise SystemError(e)

    def cs_frame_hide(self):
        """Hide frame lines of user reference system.

        Raises:
            SystemError
        """
        proj_elem_params                  = ProjectionElementParameters()
        proj_elem_params.projection_group = self.__coordinate_system + "_frame"
        proj_elem_params.figure_type      = "polyline"
        
        try:
            for frame_id in self.projection_element.frame_ids:
                proj_elem_params.figure_name = frame_id
                self.hide_proj_elem(proj_elem_params)

        except SystemError as e:
            raise SystemError(e)

    @property        
    def projector_IP(self):
        """str: Get or set projector IP address."""
        return self.__projector_IP

    @projector_IP.setter
    def projector_IP(self, IP_address):
        self.__projector_IP = IP_address
    
    @property
    def server_IP(self):
        """str: Get or set IP address of server running at projector device."""
        return self.__server_IP    
    
    @server_IP.setter
    def server_IP(self, IP_address):
        self.__server_IP = IP_address
    
    @property
    def connection_port(self):
        """int: Get or set connection port number."""
        return self.__connection_port    

    @connection_port.setter
    def connection_port(self, port):
        self.__connection_port = port
    
    @property
    def license_path(self):
        """str: Get or set license file path."""
        return self.__license_path    
    
    @license_path.setter
    def license_path(self, path):
        self.__license_path = path