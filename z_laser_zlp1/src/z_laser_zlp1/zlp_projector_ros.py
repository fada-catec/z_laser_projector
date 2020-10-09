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

import rospy
import rospkg

from z_laser_zlp1.zlp_projector_manager import ZLPProjectorManager
from z_laser_zlp1.zlp_utils import CoordinateSystemParameters, ProjectionElementParameters

from geometry_msgs.msg import Point
from std_srvs.srv import Trigger, TriggerResponse
from z_laser_zlp1.msg import Line, Curve, Text
from z_laser_zlp1.srv import CoordinateSystem, CoordinateSystemResponse
from z_laser_zlp1.srv import CoordinateSystemName, CoordinateSystemNameResponse
from z_laser_zlp1.srv import CoordinateSystemShow, CoordinateSystemShowResponse
from z_laser_zlp1.srv import CoordinateSystemList, CoordinateSystemListResponse
from z_laser_zlp1.srv import ProjectionElement, ProjectionElementResponse

class ZLPProjectorROS(object):
    """This class initilizes the services and implements the functionalities of the projection_node.

    Attributes:
        lic_path (str): folder path where Z-Laser license file is located
        projector (object): ZLPProjectorManager object from projector_manager library
    """
    def __init__(self, projector_IP, server_IP, connection_port, license_path):
        """Initialize the ZlpRos object."""

        self.lic_path = license_path

        self.projector = ZLPProjectorManager(projector_IP, server_IP, connection_port, self.lic_path)

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
        self.show_cs       = rospy.Service('show_active_coordinate_system', CoordinateSystemShow, self.show_coord_sys_cb)

        self.hide_figure   = rospy.Service('hide_figure', ProjectionElement, self.hide_figure_cb)
        self.unhide_figure = rospy.Service('unhide_figure', ProjectionElement, self.unhide_figure_cb)
        self.remove_figure = rospy.Service('remove_figure', ProjectionElement, self.remove_figure_cb)
        
        self.add_line      = rospy.Subscriber("add_line", Line, self.add_line_cb)
        self.add_curve     = rospy.Subscriber("add_curve", Curve, self.add_curve_cb)
        self.add_text      = rospy.Subscriber("add_text", Text, self.add_text_cb)

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
        """Callback of ROS service to start projection of elements associated to the active coordinate system 
        (see :func:`~projection_node.ZlpRos.set_coord_sys_cb`) on the surface.

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
            return TriggerResponse(True, "Projection stopped.")

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
        
        try:
            self.projector.define_coordinate_system(req)
            
            rospy.set_param('coordinate_system_name', req.name)
            rospy.set_param('coordinate_system_distance', req.distance)
            rospy.set_param('P1/x', req.P1.x)
            rospy.set_param('P1/y', req.P1.y)
            rospy.set_param('T1/x', req.T1.x)
            rospy.set_param('T1/y', req.T1.y)

            self.projector.cs_frame_create(req)
            self.projector.cs_axes_create(req)
            message = "Coordinate system correctly defined:"
            rospy.loginfo(message)
            T = self.get_user_coordinate_system(req.name)
            
            rospy.loginfo("Projecting demonstration")
            self.projector.show_coordinate_system()
            rospy.sleep(5)
            self.projector.hide_coordinate_system()
            self.projector.show_frame()
            rospy.sleep(5)
            self.projector.hide_frame()

            return CoordinateSystemResponse(T, True, message)

        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemResponse([], False, str(e))

    def get_coord_sys_list_cb(self,req):
        """Callback of ROS service to get the list of defined coordinate systems.
        
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Received request to get the coordinate system list at projector")
        
        cs_list = []
        try:
            cs_list = self.projector.get_coordinate_systems()
            return CoordinateSystemListResponse(True,"Coordinate system list:",cs_list)

        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemListResponse(False,str(e),cs_list)

    def set_coord_sys_cb(self,req):
        """Callback of ROS service to set the indicated coordinate system as 'active coordinate system'.
        It means that services as projection_start or show_active_coordinate_system, ..., automatically use this 
        active coordinate system to perform their task.
        The rest of coordinate systems defined and stored in the projector, stay on background until any is set as active again.

        Args:
            req (object): object with the necessary info to identify a coordinate system
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Received request to set coordinate system.")
        if not req.name:
            return CoordinateSystemNameResponse(False,"Please, specify name")

        try:
            self.projector.set_coordinate_system(req.name)

            coordinate_system_params = self.projector.get_coordinate_system_params(req.name)
            rospy.set_param('coordinate_system_name', coordinate_system_params.name)
            rospy.set_param('coordinate_system_distance', coordinate_system_params.distance)
            rospy.set_param('P1/x', coordinate_system_params.P1.x)
            rospy.set_param('P1/y', coordinate_system_params.P1.y)
            rospy.set_param('T1/x', coordinate_system_params.T1.x)
            rospy.set_param('T1/y', coordinate_system_params.T1.y)
            
            return CoordinateSystemNameResponse(True,"Set coordinate system")
                    
        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemNameResponse(False,str(e))

    def show_coord_sys_cb(self,req):
        """Callback of ROS service to project reference points, origin axes and frame of active 
        coordinate system.

        Args:
            req (object): object with the necessary info to identify a coordinate system
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Request to project active coordinate system.")
        if req.secs == 0:
            return CoordinateSystemShowResponse(False,"Please, specify seconds")

        try:
            rospy.loginfo("Projecting demonstration")
            self.projector.show_coordinate_system()
            rospy.sleep(req.secs)
            self.projector.hide_coordinate_system()
            self.projector.show_frame()
            rospy.sleep(req.secs)
            self.projector.hide_frame()
            return CoordinateSystemShowResponse(True,"Coordinate system showed")
        
        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemShowResponse(False,str(e))

    def remove_coord_sys_cb(self,req):
        """Callback of ROS service to remove active coordinate system.

        Args:
            req (object): object with the necessary info to identify a coordinate system
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        rospy.loginfo("Received request to remove coordinate system")
        if not req.name:
            return CoordinateSystemNameResponse(False,"Please, specify name")
        
        try:
            self.projector.remove_coordinate_system(req.name)
            return CoordinateSystemNameResponse(True,"Removed coordinate system")
        
        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemNameResponse(False,str(e))
    
    def add_line_cb(self,msg):
        """Callback of ROS topic to define a new polyline projection element associated to the active coordinate system.

        Args:
            msg (object): object with the necessary info to define a new polyline
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an 
            information message string
        """
        rospy.loginfo("Received request to add a new line to the active coordinate system.")

        if not msg.projection_group or not msg.figure_name:
            return ProjectionElementResponse(False,"projection_group or figure_name request is empty.")

        try:
            self.projector.create_polyline(msg)
            rospy.loginfo("Line added correctly.")
            return ProjectionElementResponse(True,"Line added correctly.")
        
        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(False,str(e))

    def add_curve_cb(self,msg):
        """Callback of ROS topic to define a new curve projection element associated to the active coordinate system.

        Args:
            msg (object): object with the necessary info to define a new curve
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an 
            information message string
        """
        rospy.loginfo("Received request to add a new curve to the active coordinate system.")

        if not msg.curve_type or not msg.projection_group or not msg.figure_name:
            return ProjectionElementResponse(False,"curve_type or projection_group or figure_name request is empty.")

        try:
            self.projector.create_curve(msg)
            rospy.loginfo("Curve added correctly.")
            return ProjectionElementResponse(True,"Curve added correctly.")
        
        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(False,str(e))

    def add_text_cb(self,msg):
        """Callback of ROS topic to define a new text projection element associated to the active coordinate system.

        Args:
            msg (object): object with the necessary info to define a new text
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        rospy.loginfo("Received request to add a new text to the active coordinate system.")

        if not msg.projection_group or not msg.figure_name:
            return ProjectionElementResponse(False,"projection_group or figure_name request is empty.")

        try:
            self.projector.create_text(msg)
            rospy.loginfo("Text added correctly.")
            return ProjectionElementResponse(True,"Text added correctly.")
        
        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(False,str(e))

    def hide_figure_cb(self,req):
        """Callback of ROS service to hide specific projection element from active coordinate system.

        Args:
            req (object): object with the necessary info to identify a projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        rospy.loginfo("Received request to hide figure.")

        if not req.figure_type or not req.group_name or not req.figure_name:
            return ProjectionElementResponse(False,"figure_type or group_name or figure_name request is empty")

        try:        
            self.projector.hide_proj_elem(req)
            return ProjectionElementResponse(True,"Figure hidden")

        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(False,str(e))

    def unhide_figure_cb(self,req):
        """Callback of ROS service to unhide specific projection element from active coordinate system.

        Args:
            req (object): object with the necessary info to identify a projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        rospy.loginfo("Received request to unhide figure.")

        if not req.figure_type or not req.group_name or not req.figure_name:
            return ProjectionElementResponse(False,"figure_type or group_name or figure_name request is empty")

        try:        
            self.projector.unhide_proj_elem(req)
            return ProjectionElementResponse(True,"Figure unhidden")

        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(False,str(e))

    def remove_figure_cb(self,req):
        """Callback of ROS service to remove specific figure from active coordinate system.

        Args:
            req (object): object with the necessary info to identify a projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        rospy.loginfo("Received request to remove figure.")

        if not req.figure_type or not req.group_name or not req.figure_name:
            return ProjectionElementResponse(False,"figure_type or group_name or figure_name request is empty")

        try:        
            self.projector.remove_proj_elem(req)
            return ProjectionElementResponse(True,"Figure removed")

        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(False,str(e))

    def read_coordinate_system(self):
        """Read necessary info to define a new coordinate system from rosparams.

        Returns:
            cs_params (object): object with the necessary info to define a new coordinate system
        """
        rospy.loginfo("Reading coordinate system data")
        cs_params            = CoordinateSystemParameters()
        cs_params.name       = rospy.get_param('coordinate_system_name', "default_cs")
        cs_params.resolution = rospy.get_param('coordinate_system_resolution', 1000)
        cs_params.distance   = rospy.get_param('coordinate_system_distance', 1500)
        cs_params.P1.x       = rospy.get_param('P1/x', -100)
        cs_params.P1.y       = rospy.get_param('P1/y', -100)
        cs_params.P2.x       = rospy.get_param('P2/x', -100)
        cs_params.P2.y       = rospy.get_param('P2/y',  100)
        cs_params.P3.x       = rospy.get_param('P3/x',  100)
        cs_params.P3.y       = rospy.get_param('P3/y',  100)
        cs_params.P4.x       = rospy.get_param('P4/x',  100)
        cs_params.P4.y       = rospy.get_param('P4/y', -100)
        cs_params.T1.x       = rospy.get_param('T1/x',    0)
        cs_params.T1.y       = rospy.get_param('T1/y',    0)
        return cs_params

    def get_user_coordinate_system(self,coordinate_system_name):
        """Get the parameters values of a defined coordinate system.

        Returns:
            list: list with the (x,y) position of points T1,T2,T3 and T4 from user reference system {T}
        """
        cs_params = self.projector.get_coordinate_system_params(coordinate_system_name)
        T1 = Point(cs_params.T1.x, cs_params.T1.y, cs_params.distance)
        T2 = Point(cs_params.T2.x, cs_params.T2.y, cs_params.distance)
        T3 = Point(cs_params.T3.x, cs_params.T3.y, cs_params.distance)
        T4 = Point(cs_params.T4.x, cs_params.T4.y, cs_params.distance)
        rospy.loginfo("[T] Reference points: \nT1: \n{}, \nT2: \n{}, \nT3: \n{}, \nT4: \n{}".format(T1,T2,T3,T4))

        return [T1,T2,T3,T4]

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
            return True

    def initialize_coordinate_system(self):
        """Initialize the factory or user predefined coordinate system at ros projection_node launch."""
        cs_params = self.read_coordinate_system()
        try:
            self.projector.define_coordinate_system(cs_params)
            self.projector.cs_frame_create(cs_params)
            self.projector.cs_axes_create(cs_params)
            rospy.loginfo("Coordinate System [{}] loaded".format(cs_params.name))
            self.get_user_coordinate_system(cs_params.name)

            rospy.loginfo("Projecting demonstration")
            self.projector.show_coordinate_system()
            rospy.sleep(3)
            self.projector.hide_coordinate_system()
            self.projector.show_frame()
            rospy.sleep(3)
            self.projector.hide_frame()            

        except Exception as e:
            if "InvalidRelativePath" in str(e):
                rospy.logwarn("Possible error: check coordinate system name is not empty")
            rospy.logerr(e)

    def shutdown_handler(self):
        """Handler to close connection when node exits.
 
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
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