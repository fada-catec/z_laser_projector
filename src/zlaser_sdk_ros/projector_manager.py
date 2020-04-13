#!/usr/bin/env python3

import sys
import os
import threading
import time
import math
import zlp
from zlp import ProjectorClient, CoordinateSystem, ProjectionElementControl

class ProjectorManager:
    """This class implement the functions to manage the ZLP projector"""
    
    def __init__(self, projector_IP = "192.168.10.10", server_IP = "192.168.10.11", connection_port = 9090):
        """Initialize the ProjectorManager object.
        
            Args:
                string projector_IP: projector IP address
                string server_IP: server IP address
                string connection_port: number of connection port"""
        self.projector_IP = projector_IP
        self.server_IP = server_IP
        self.connection_port = connection_port
        self.license_path = "/lic/1900027652.lic" #???
        self.projector_id = ""

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

    def load_license(self):
        success,message = self.projector_client.transfer_license(self.license_path)
        return success,message

    def check_license(self):
        success = self.projector_client.check_license()
        return success

    def geotree_operator_create(self):
        module_id,success,message = self.projector_client.function_module_create()
        thrift_client = self.projector_client.get_thrift_client()

        self.cs_element = CoordinateSystem(self.projector_id, module_id, thrift_client)
        self.projection_element = ProjectionElementControl(module_id,thrift_client)

        return success,message

    def start_projection(self,coord_sys):
        success,message = self.projector_client.start_project(coord_sys)
        return success,message 

    def stop_projection(self): 
        success,message = self.projector_client.stop_project()
        return success,message

    def get_coordinate_systems(self):
        cs_list,success,message = self.cs_element.coordinate_system_list()
        return cs_list,success,message

    def define_coordinate_system(self,req):
        coord_sys,success,message = self.cs_element.define_cs(req)
        return coord_sys,success,message

    def set_coordinate_system(self,coord_sys): 
        success,message = self.cs_element.set_cs(coord_sys)
        return success,message

    def register_coordinate_system(self,coord_sys):
        success,message = self.cs_element.register_cs(coord_sys)
        return success,message

    def show_coordinate_system(self,coord_sys,secs):
        success,message = self.cs_element.show_cs(coord_sys, secs)
        return success,message

    def remove_coordinate_system(self,coord_sys):
        success,message = self.cs_element.remove_cs(coord_sys)
        return success,message

    def create_polyline(self,coord_sys,projection_group,id,x,y,angle,r):
        success,message = self.projection_element.define_polyline(coord_sys,projection_group,id,x,y,angle,r)
        return success,message

    def hide_shape(self,projection_group,shape_name,id): 
        success,message = self.projection_element.deactivate_shape(projection_group,shape_name,id)
        return success,message

    def unhide_shape(self,projection_group,shape_name,id): 
        success,message = self.projection_element.reactivate_shape(projection_group,shape_name,id)
        return success,message

    def remove_shape(self,projection_group,shape_name,id):
        success,message = self.projection_element.delete_shape(projection_group,shape_name,id)
        return success,message
