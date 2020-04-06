#!/usr/bin/env python3

import sys
import os
import threading
import time
#from zlaser_sdk_ros import zlp
import zlp
import math

from zlp import ProjectorClient, CoordSys, ProjectionElement

class ProjectorManager:
    """This is a class to operate the ZLP projector. """
    # https://www.python.org/dev/peps/pep-0257/#multi-line-docstrings
    # http://google.github.io/styleguide/pyguide.html
    
    def __init__(self):
        """Initialize the ProjectorManager object."""
        # Define default values
        self.projector_IP = "192.168.10.10"
        self.server_IP = "192.168.10.11"
        self.connection_port = 9090
        self.license_path = "/lic/1900027652.lic" #???
        
        # Auxiliar variables
        # self.projector_id = ""
        # self.module_id = ""
        # self.reference_object_list = []
        # self.coordinate_system = ""
        # self.geo_tree_elements = []

        self.projector_client = ProjectorClient(self.projector_IP,self.server_IP,self.connection_port) # Create ThriftClient object
        # self.cs_element = CoordSys(projector_id, module_id)
        # self.projection_element = ProjectionElement(pmodule_id)

    def client_server_connect(self):
        """Create the client and connect to thrift server (located at projector) of ZLP-Service.

            Returns:
                string: message """
        
        try:
            self.projector_client.connect()
            return "Connected to server. You can activate projector now"
        except Exception as e:
            return e

    def client_server_disconnect(self):
        """Disconnect client from ZLP-Service thrift server and close own event server.
        
            Returns:
                string: message """

        try:
            self.projector_client.disconnect()
            return "Disconnected to server. End of connection"
        except Exception as e:
            return e

    def activate(self):
        """Set properties to activate projector.
        
            Returns:
                string: message """

        try:
            self.projector_client.activate_projector()
            return "Projector activated. You can start the projection now"
        except Exception as e:
            return e

    def deactivate(self):
        """Set properties to deactivate projector.
        
            Returns:
                string: message """
        
        try:
            self.projector_client.deactivate_projector()
            return "Projector deactivated."
        except Exception as e:
            return e

    def load_license(self):
        """Transfer license file to projector.
        
            Returns:
                string: message """

        try:
            self.projector_client.transfer_license(self.license_path)
            return "License transfered"
        except Exception as e:
            return e

    def check_license(self):
        """Check if license is valid.
        
            Returns:
                bool: return True for success, False otherwise """
        
        return self.projector_client.check_license()

    def function_module_create(self):
        """Create function module to operate with GeoTreeElements (coordinate systems and shapes).
        
            Returns:
                string: message """

        try:
            module_id, projector_id = self.projector_client.function_module_create()
            
            print("CoordSys() from projector_manager")
            self.cs_element = CoordSys(projector_id, module_id)
            # print("ProjectionElement() from projector_manager")
            # self.projection_element = ProjectionElement(module_id)

            return "Function Module Created"
        except Exception as e:
            return e

    def start_projection(self,coord_sys):
        """Start projection on the surface of all figures (shapes) that belong to the active coordinate system.
            
            Returns:
                string: message """
        
        return self.projector_client.start_project(coord_sys)

    def stop_projection(self): # este tipo de método son los que se incluyen en zlp pero este concretamente no está incluido, se añade aqui para no tocar zlp
        """Stop projection of all figures.
            
            Returns:
                string: message """
        
        return self.projector_client.stop_project()






    def get_coordinate_systems(self):
        """Get list of current available coordinate systems from projector.
        
            Returns:
                list: names list of available coordinate systems (strings) """
        cs_list = self.cs_element.coordinate_system_list()
        print("BBBBBBBBBBBBBBBBBB")
        return cs_list

    def define_coordinate_system(self,req):
        """Generate new coordinate system.

            Args:
                struct (req): structure with the necessary parameters value to generate the coordinate system
            
            Returns:
                string: name of the coordinate system generated """

        coord_sys = self.cs_element.define_cs(req)
        print("Reference object created. Coordinate system defined but not registered.")
        return(coord_sys)

    def set_coordinate_system(self,coord_sys): 
        """Activate the new generated coordinate system and deactivate the other existing coordinate systems at the projector.

            Args:
                string (coord_sys): name of the new coordinate system
            
            Returns:
                string: message """
        
        return self.cs_element.set_cs(coord_sys)

    def register_coordinate_system(self,coord_sys):
        """Register the new coordinate system at the projector once it has been generated and activated.

            Args:
                string (coord_sys): name of the coordinate system
            
            Returns:
                string: message """

        return self.cs_element.register_cs(coord_sys)

    def show_coordinate_system(self,coord_sys,secs):
        """Project on the surface an existing coordinate system.

            Args:
                string (coord_sys): name of the coordinate system
                int (secs): number of seconds the projection lasts

            Returns:
                string: message """

        return self.cs_element.show_cs(coord_sys, secs)

    def remove_coordinate_system(self,coord_sys):
        """Delete a coordinate system.
            Args:
                string (coord_sys): name of the coordinate system
                
            Returns:
                string: message """

        return self.cs_element.remove_cs(coord_sys)






    def create_polyline(self,coord_sys,projection_group,id,x,y,angle,r):
        """Create a new line to project.

            Args:
                string (projection_group): name of the projection group
                string (id): name of the shape identificator
                float (x): x-axis value of the initial point position 
                float (y): y-axis value of the initial point position 
                float (angle): line angle value
                float (r): length of the line
                int (secs): number of seconds the projection lasts
                
            Returns:
                string: message """
        
        return self.projection_element.define_polyline(coord_sys,projection_group,id,x,y,angle,r)

    def hide_shape(self,projection_group,shape_name,id): # deactivate shape
        """Hide (deactivate) a figure from a group of the active coordinate system.

            Args:
                string (projection_group): name of the projection group
                string (shape_name): type of figure (polyline, circle, etc.)
                string (id): name of the shape identificator
                
            Returns:
                string: message """
        
        return self.projection_element.deactivate_shape(projection_group,shape_name,id)

    def unhide_shape(self,projection_group,shape_name,id): # deactivate shape
        """Unhide (activate) a figure from a group of the active coordinate system.

            Args:
                string (projection_group): name of the projection group
                string (shape_name): type of figure (polyline, circle, etc.)
                string (id): name of the shape identificator
                
            Returns:
                string: message """
        
        return self.projection_element.reactivate_shape(projection_group,shape_name,id)

    def remove_shape(self,projection_group,shape_name,id):
        """Delete a figure from the active coordinate system.

            Args:
                string (projection_group): name of the projection group
                string (shape_name): type of figure (polyline, circle, etc.)
                string (id): name of the shape identificator
                
            Returns:
                string: message """
        
        return self.projection_element.delete_shape(projection_group,shape_name,id)
