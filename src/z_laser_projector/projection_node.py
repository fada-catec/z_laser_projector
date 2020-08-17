#!/usr/bin/env python3

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

"""Central node that provides a collection of services that allows to operate and control the ZLP1 laser projector and simplify the task of 
developing further advanced features."""

import sys
import rospy
import rospkg
import os
import time
import numpy as np

# from z_laser_projector.projector_manager import ProjectorManager # noqa?
# from z_laser_projector.utils import CoordinateSystemParameters, ProjectionElementParameters # noqa?
from projector_manager import ProjectorManager
from utils import CoordinateSystemParameters, ProjectionElementParameters

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
from z_laser_projector.msg import Line, ReferencePoint
from z_laser_projector.srv import CoordinateSystem, CoordinateSystemResponse
from z_laser_projector.srv import CoordinateSystemName, CoordinateSystemNameResponse
from z_laser_projector.srv import CoordinateSystemList, CoordinateSystemListResponse
from z_laser_projector.srv import ProjectionElement, ProjectionElementResponse

class ProjectionNode:
    """This class initilizes the services and implements the functionalities of the projection_node.
    
    Args:
        load_cs (str): node launcher parameter to load a user predefined coordinate system from template in the 
            initilization. "true" to load user template, "false" otherwise.

    Attributes:
        lic_path (str): folder path where Z-Laser license file is located
        projector (object): ProjectorManager object from projector_manager library
    """
    def __init__(self,load_cs):
        """Initialize the ProjectionNode object."""
        rospy.init_node('projection_node')

        projector_IP    = rospy.get_param('projector_IP', "192.168.10.10") 
        server_IP       = rospy.get_param('server_IP', "192.168.10.11") 
        connection_port = rospy.get_param('connection_port', 9090) 
        license_file    = rospy.get_param('license_file', "1900027652.lic") 

        # define license file path
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('z_laser_projector')
        self.lic_path = pkg_path + "/lic/" + license_file
        # Create ProjectorManager instance
        self.projector = ProjectorManager(projector_IP, server_IP, connection_port, self.lic_path)
        # Connect, load license and activate projector
        error = self.setup_projector()
        # If set by user, create a coordinate system
        if not error:
            self.initialize_coordinate_system(load_cs)

        # Create services to interact with projector
        self.open_services()
        # Create handler to close connection when exiting node
        rospy.on_shutdown(self.shutdown_handler)

        rospy.spin()

    def open_services(self):
        """Open ROS services that allow projector device control."""
        self.connect       = rospy.Service('connect', Trigger, self.connection_cb)
        self.disconnect    = rospy.Service('disconnect', Trigger, self.disconnection_cb)
        self.start_proj    = rospy.Service('projection_start', Trigger, self.projection_start_cb)
        self.stop_proj     = rospy.Service('projection_stop', Trigger, self.projection_stop_cb)

        self.manual_cs     = rospy.Service('define_coordinate_system', CoordinateSystem, self.manual_define_coord_sys_cb)

        self.get_cs_list   = rospy.Service('coordinate_system_list', CoordinateSystemList, self.get_coord_sys_list_cb)

        self.set_cs        = rospy.Service('set_coordinate_system', CoordinateSystemName, self.set_coord_sys_cb)
        self.rem_cs        = rospy.Service('remove_coordinate_system', CoordinateSystemName, self.remove_coord_sys_cb)
        self.show_cs       = rospy.Service('show_current_coordinate_system', CoordinateSystemName, self.show_coord_sys_cb)

        self.hide_shape    = rospy.Service('hide_shape', ProjectionElement, self.hide_shape_cb)
        self.unhide_shape  = rospy.Service('unhide_shape', ProjectionElement, self.unhide_shape_cb)
        self.remove_shape  = rospy.Service('remove_shape', ProjectionElement, self.remove_shape_cb)
        
        self.add_line      = rospy.Subscriber("add_line", Line, self.add_line_cb)

        rospy.loginfo("Use ROS Services: \n\t\t\trosservice list")

    def connection_cb(self,req):
        """Callback of ROS service to connect to ZLP-Service, transfer license and activate projector.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an 
            information message string
        """
        rospy.loginfo("Received request to connect projector.")
        try:
            self.projector.client_server_connect()       
            self.projector.load_license(self.lic_path)
            self.projector.activate()
            self.projector.geotree_operator_create()
            rospy.loginfo("Projector connected.")
            return TriggerResponse(True, "Projector connected.")
        
        except Exception as e:
            rospy.logerr(e)
            return TriggerResponse(False, str(e))

    def disconnection_cb(self,req):
        """Callback of ROS service to deactivate projector and disconnect from ZLP-Service.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Received request to disconnect projector.")
        try:
            self.projector.deactivate()
            self.projector.client_server_disconnect()
            rospy.loginfo("Projector disconnected.")
            return TriggerResponse(True, "Projector disconnected.")

        except Exception as e:
            rospy.logerr(e)
            return TriggerResponse(False, str(e))

    def projection_start_cb(self,req):
        """Callback of ROS service to start projection of elements associated to the current operating coordinate system 
        (see :func:`~projection_node.ProjectionNode.set_coord_sys_cb`) on the surface.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Received request to start projection")
        try:
            self.projector.start_projection()
            return TriggerResponse(True, "Projection started.")

        except Exception as e:
            rospy.logerr(e)
            return TriggerResponse(False, str(e))
    
    def projection_stop_cb(self,req):
        """Callback of ROS service to stop projection of all elements.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Received request to stop projection")
        try:
            self.projector.stop_projection()
            return TriggerResponse(True, "Projection stoped.")

        except Exception as e:
            rospy.logerr(e)
            return TriggerResponse(False, str(e))

    def manual_define_coord_sys_cb(self,req):
        """Callback of ROS service to define a new reference system, stating the points coordinates manually by the user.

        Args:
            req (object): object with the necessary info to define a new coordinate system
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Received request to create a new coordinate system manually. Please wait for the system to indicate the end.")
        
        cs_params = CoordinateSystemParameters()
        cs_params.set_request_params(req)
        try:
            self.projector.define_coordinate_system(cs_params)
            self.projector.cs_frame_create(cs_params)
            self.projector.cs_axes_create(cs_params)
            self.projector.show_coordinate_system(5)
            message = "Coordinate system correctly defined"
            rospy.loginfo("Coordinate system correctly defined")
            return CoordinateSystemResponse(Bool(True),String(message))

        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemResponse(Bool(False), String(str(e)))

    def get_coord_sys_list_cb(self,req):
        """Callback of ROS service to get the list of defined coordinate systems.
        
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Received request to get the coordinate system list at projector")
        
        cs_list = []
        cs_list = [String(cs_name) for cs_name in cs_list]
        try:
            cs_list = self.projector.get_coordinate_systems()
            cs_list = [String(cs_name) for cs_name in cs_list]
            return CoordinateSystemListResponse(Bool(True),String("Coordinate system list:"),cs_list)

        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemListResponse(Bool(False),String(str(e)),cs_list)

    def set_coord_sys_cb(self,req):
        """Callback of ROS service to set the indicated coordinate system as 'current operating coordinate system'.
        It means that services as projection_start or show_current_coordinate_system, ..., automatically use this 
        'current operating coordinate system' to perform their task.
        The rest of coordinate systems defined and stored in the projector, stay on background until any is set again.

        Args:
            req (object): object with the necessary info to identify a coordinate system
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Received request to set coordinate system.")
        if not req.cs_name.data:
            return CoordinateSystemNameResponse(Bool(False),String("Please, specify cs_name"))

        try:
            self.projector.set_coordinate_system(req.cs_name.data)
            return CoordinateSystemNameResponse(Bool(True),String("Set coordinate system"))
                    
        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemNameResponse(Bool(False),String(str(e)))

    def show_coord_sys_cb(self,req):
        """Callback of ROS service to project reference points, origin axes and frame of current operating 
        coordinate system.

        Args:
            req (object): object with the necessary info to identify a coordinate system
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Request to project current coordinate system.")
        if req.secs.data == 0:
            return CoordinateSystemNameResponse(Bool(False),String("Please, specify seconds"))

        try:
            self.projector.show_coordinate_system(req.secs.data)
            return CoordinateSystemNameResponse(Bool(True),String("Coordinate system showed"))
        
        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemNameResponse(Bool(False),String(str(e)))

    def remove_coord_sys_cb(self,req):
        """Callback of ROS service to remove current coordinate system.

        Args:
            req (object): object with the necessary info to identify a coordinate system
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Received request to remove coordinate system")
        if not req.cs_name.data:
            return CoordinateSystemNameResponse(Bool(False),String("Please, specify cs_name"))
        
        try:
            self.projector.remove_coordinate_system(req.cs_name.data)
            return CoordinateSystemNameResponse(Bool(True),String("Removed coordinate system"))
        
        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemNameResponse(Bool(False),String(str(e)))
    
    def add_line_cb(self,msg):
        """Callback of ROS topic to define a new polyline projection element associated to the current
        operating coordinate system.

        Args:
            msg (object): object with the necessary info to define a new polyline
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an 
            information message string
        """
        rospy.loginfo("Received request to add a line to the current coordinate system.")

        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.set_line_params(msg)
        
        if not msg.group_name.data or not msg.shape_id.data:
            return ProjectionElementResponse(Bool(False),String("group_name or shape_id request is empty."))

        try:
            self.projector.create_polyline(proj_elem_params)
            rospy.loginfo("Line added correctly.")
            return ProjectionElementResponse(Bool(True),String("Line added correctly."))
        
        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(Bool(False),String(str(e)))

    def hide_shape_cb(self,req):
        """Callback of ROS service to hide specific figure from current coordinate system.

        Args:
            req (object): object with the necessary info to identify a projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Received request to hide shape.")

        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.set_request_params(req)

        if not req.shape_type.data or not req.group_name.data or not req.shape_id.data:
            return ProjectionElementResponse(Bool(False),String("shape_type or group_name or shape_id request is empty"))

        try:        
            self.projector.hide_shape(proj_elem_params)
            return ProjectionElementResponse(Bool(True),String("Hide shape"))

        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(Bool(False),String(str(e)))

    def unhide_shape_cb(self,req):
        """Callback of ROS service to unhide specific figure from current coordinate system.

        Args:
            req (object): object with the necessary info to identify a projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Received request to unhide shape.")

        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.set_request_params(req)

        if not req.shape_type.data or not req.group_name.data or not req.shape_id.data:
            return ProjectionElementResponse(Bool(False),String("shape_type or group_name or shape_id request is empty"))

        try:        
            self.projector.unhide_shape(proj_elem_params)
            return ProjectionElementResponse(Bool(True),String("Hide shape"))

        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(Bool(False),String(str(e)))

    def remove_shape_cb(self,req):
        """Callback of ROS service to remove specific figure from current coordinate system.

        Args:
            req (object): object with the necessary info to identify a projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Received request to remove shape.")

        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.set_request_params(req)

        if not req.shape_type.data or not req.group_name.data or not req.shape_id.data:
            return ProjectionElementResponse(Bool(False),String("shape_type or group_name or shape_id request is empty"))

        try:        
            self.projector.remove_shape(proj_elem_params)
            return ProjectionElementResponse(Bool(True),String("Hide shape"))

        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(Bool(False),String(str(e)))

    def read_coordinate_system(self):
        """Read necessary info to define a new coordinate system from template (.yaml).

        Returns:
            cs_params (object): object with the necessary info to define a new coordinate system
        """
        rospy.loginfo("Reading coordinate system data")
        cs_params = CoordinateSystemParameters()
        cs_params.name        = rospy.get_param('coordinate_system_name', "default_cs")
        cs_params.resolution  = rospy.get_param('coordinate_system_resolution', 1000)
        cs_params.d           = rospy.get_param('coordinate_system_distance', 1500)
        cs_params.P1_x        = rospy.get_param('P1/x', -100)
        cs_params.P1_y        = rospy.get_param('P1/y', -100)
        cs_params.P2_x        = rospy.get_param('P2/x', -100)
        cs_params.P2_y        = rospy.get_param('P2/y',  100)
        cs_params.P3_x        = rospy.get_param('P3/x',  100)
        cs_params.P3_y        = rospy.get_param('P3/y',  100)
        cs_params.P4_x        = rospy.get_param('P4/x',  100)
        cs_params.P4_y        = rospy.get_param('P4/y', -100)
        cs_params.T1_x        = rospy.get_param('T1/x',    0)
        cs_params.T1_y        = rospy.get_param('T1/y',    0)
        return cs_params

    def setup_projector(self):
        """Setup projector at initialization (connect to ZLP-Service, transfer license file and activate projector).
 
        Returns:
            bool: success value. True if success, False otherwise.
        """
        rospy.loginfo("Setting projector up")
        try:
            self.projector.connect_and_setup()
            rospy.loginfo("Projector connected.")
            return False
        
        except Exception as e:
            rospy.logerr(e)

    def initialize_coordinate_system(self,load_cs):
        """Initialize the factory or user predefined coordinate system at ros projection_node launch.

        Args:
            load_cs (str): node launcher parameter to load in the initilization an user predefined coordinate system 
                from template if it is "true", or factory coordinate system if is "false".
        """
        cs_params = self.read_coordinate_system()
        try:
            self.projector.define_coordinate_system(cs_params)
            self.projector.cs_frame_create(cs_params)
            self.projector.cs_axes_create(cs_params)
            self.projector.show_coordinate_system(5)
            
            if load_cs == "true":
                rospy.loginfo("User defined coordinate system loaded: {}".format(cs_params.name))
            else:
                rospy.loginfo("Factory default coordinate system loaded: {}".format(cs_params.name))
                rospy.logwarn("Defining a new coordinate system by user is highly recommended.")

        except Exception as e:
            rospy.logerr(e)

    def shutdown_handler(self):
        """Handler to close connection when node exits.
 
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Disconnecting before shutdown...")
        try:
            self.projector.deactivate()
            self.projector.client_server_disconnect()
            rospy.loginfo("Projector disconnected.")
            return TriggerResponse(True, "Projector disconnected.")

        except Exception as e:
            rospy.logerr(e)
            return TriggerResponse(False, str(e))

def main():
    """Init ROS node"""
    ProjectionNode(sys.argv[1])

if __name__ == '__main__':
    main()