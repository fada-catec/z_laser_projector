#!/usr/bin/env python3

import sys
import time
import math
from zlaser_sdk_ros.zlp import ProjectorClient, CoordinateSystem, ProjectionElementControl
from zlaser_sdk_ros.zlp import CoordinateSystemParameters, ProjectionElementParameters

class ProjectorManager:
    
    def __init__(self, projector_IP = "192.168.10.10", server_IP = "192.168.10.11", connection_port = 9090):
        self.projector_IP = projector_IP
        self.server_IP = server_IP
        self.connection_port = connection_port
        self.projector_id = ""
        self.coordinate_system = ""

        self.projector_client = ProjectorClient() 

    def client_server_connect(self):
        success,message = self.projector_client.connect(self.server_IP, self.connection_port)
        return success,message

    def client_server_disconnect(self):
        success,message = self.projector_client.disconnect()
        return success,message

    def activate(self):
        self.projector_id,success,message = self.projector_client.activate_projector(self.projector_IP)
        return success,message

    def deactivate(self):
        success,message = self.projector_client.deactivate_projector()
        return success,message

    def load_license(self,license_path):
        self.projector_client.transfer_license(license_path)
        success,message = self.projector_client.check_license()
        return success,message

    def geotree_operator_create(self):
        module_id,success,message = self.projector_client.function_module_create()
        thrift_client = self.projector_client.get_thrift_client()

        self.cs_element = CoordinateSystem(self.projector_id, module_id, thrift_client)
        self.projection_element = ProjectionElementControl(module_id,thrift_client)

        if not self.coordinate_system:
            message = message + "\nNo Current Coordinate System set so far."

        return success,message

    def start_projection(self):
        success,message = self.projector_client.start_project(self.coordinate_system)
        return success,message 

    def stop_projection(self): 
        success,message = self.projector_client.stop_project()
        return success,message

    def get_coordinate_systems(self):
        cs_list,success,message = self.cs_element.coordinate_system_list()
        return cs_list,success,message

    def define_coordinate_system(self,cs_params):
        coord_sys,success,message = self.cs_element.define_cs(cs_params)
        if success:
            success,message = self.cs_element.register_cs(coord_sys)
            if success:
                success,message = self.set_coordinate_system(coord_sys)
                message = "Coordinate system defined, registered and set."

        return success,message

    def set_coordinate_system(self,coord_sys): 
        success,message = self.cs_element.set_cs(coord_sys)
        if success:
            self.coordinate_system = coord_sys
        return success,message

    def show_coordinate_system(self,secs):
        if not self.coordinate_system:
            success = False
            message = "Coordinate system cannot be showed because there is none or the current is not set first."
            message = message + "\n NOTE: Check if the coordinate system is set first."
            return success,message
        
        success,message = self.cs_element.show_cs(self.coordinate_system, secs)
        return success,message

    def remove_coordinate_system(self,coord_sys):
        success,message = self.cs_element.remove_cs(coord_sys)
        if success:
            self.coordinate_system = ""
            message = "Coordinate system removed. Define or set new one."

        return success,message

    def create_polyline(self,proj_elem_params):
        if not self.coordinate_system:
            success = False
            message = "There is not a current coordinate system. Define or set one first."
            return success,message

        success,message = self.projection_element.define_polyline(self.coordinate_system,proj_elem_params)
        return success,message

    def hide_shape(self,proj_elem_params): 
        success,message = self.projection_element.deactivate_shape(proj_elem_params)
        return success,message

    def unhide_shape(self,proj_elem_params): 
        success,message = self.projection_element.reactivate_shape(proj_elem_params)
        return success,message

    def remove_shape(self,proj_elem_params):
        success,message = self.projection_element.delete_shape(proj_elem_params)
        return success,message

    def cs_axes_create(self,cs_params):
        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.shape_type            = "polyline"
        proj_elem_params.projection_group_name = self.coordinate_system + "_origin"
        proj_elem_params.x                     = cs_params.T1_x
        proj_elem_params.y                     = cs_params.T1_y
        proj_elem_params.length                = 100
        
        proj_elem_params.shape_id              = "axis_x"
        proj_elem_params.angle                 = 0

        success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params) 
        if success:
            proj_elem_params.shape_id = "axis_y"
            proj_elem_params.angle    = 90
            success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params)
        
        if success:
            proj_elem_params.x        = cs_params.T1_x + 100
            proj_elem_params.y        = cs_params.T1_y
            proj_elem_params.length   = 30
            proj_elem_params.shape_id = "axis_x_arrow1"
            proj_elem_params.angle    = 180 - 15
            success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params)
        
        if success:
            proj_elem_params.shape_id = "axis_x_arrow2"
            proj_elem_params.angle    = 180 + 15
            success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params)
        
        if success:
            proj_elem_params.x        = cs_params.T1_x
            proj_elem_params.y        = cs_params.T1_y + 100
            proj_elem_params.length   = 20
            proj_elem_params.shape_id = "axis_y_arrow1"
            proj_elem_params.angle    = 270 - 15
            success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params)
        
        if success:
            proj_elem_params.shape_id = "axis_y_arrow2"
            proj_elem_params.angle    = 270 + 15
            success,message = self.projection_element.define_polyline(self.coordinate_system, proj_elem_params)
        
        if success:
            self.start_projection()
            time.sleep(5)
            self.stop_projection()
                                
        success,message = self.hide_shape(proj_elem_params)
        if success:
            proj_elem_params.shape_id = "axis_x"
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
            message = "Coordinate system origin axes created correctly"

        return success,message

    def cs_axes_unhide(self):
        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.shape_type            = "polyline"
        proj_elem_params.projection_group_name = self.coordinate_system + "_origin"
        proj_elem_params.shape_id              = "axis_x"
        
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
            self.start_projection()
            time.sleep(5)
            self.stop_projection()
                
        success,message = self.hide_shape(proj_elem_params)
        if success:
            proj_elem_params.shape_id = "axis_x"
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
            message = "Coordinate system origin axes showed correctly"

        return success,message