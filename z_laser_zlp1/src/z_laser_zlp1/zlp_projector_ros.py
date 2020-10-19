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
# from z_laser_zlp1.zlp_utils import CoordinateSystemParameters, ProjectionElementParameters

from geometry_msgs.msg import Point
from std_srvs.srv import Trigger, TriggerResponse
from z_laser_msgs.msg import Line, Curve, Text
from z_laser_msgs.srv import CoordinateSystem, CoordinateSystemRequest, CoordinateSystemResponse
from z_laser_msgs.srv import CoordinateSystemName, CoordinateSystemNameResponse
from z_laser_msgs.srv import CoordinateSystemShow, CoordinateSystemShowResponse
from z_laser_msgs.srv import CoordinateSystemList, CoordinateSystemListResponse
from z_laser_msgs.srv import ProjectionElement, ProjectionElementResponse

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
        self.auto_cs       = rospy.Service('search_targets', CoordinateSystem, self.auto_search_target_cb)

        self.get_cs_list   = rospy.Service('coordinate_system_list', CoordinateSystemList, self.get_coord_sys_list_cb)

        self.set_cs        = rospy.Service('set_coordinate_system', CoordinateSystemName, self.set_coord_sys_cb)
        self.rem_cs        = rospy.Service('remove_coordinate_system', CoordinateSystemName, self.remove_coord_sys_cb)
        self.show_cs       = rospy.Service('show_active_coordinate_system', CoordinateSystemShow, self.show_coord_sys_cb)

        self.hide_figure   = rospy.Service('hide_figure', ProjectionElement, self.hide_figure_cb)
        self.unhide_figure = rospy.Service('unhide_figure', ProjectionElement, self.unhide_figure_cb)
        self.remove_figure = rospy.Service('remove_figure', ProjectionElement, self.remove_figure_cb)
        self.monit_figure  = rospy.Service('monitor_figure', ProjectionElement, self.keyboard_monitor_figure_cb)
        
        self.add_line      = rospy.Subscriber("add_line", Line, self.add_line_cb)
        self.add_curve     = rospy.Subscriber("add_curve", Curve, self.add_curve_cb)
        self.add_text      = rospy.Subscriber("add_text", Text, self.add_text_cb)

        rospy.loginfo("Use ROS Services: \n\t\t\trosservice list")

    def open_viz_services(self):
        
        # zlaser visualizer services
        self.viz_start_proj    = rospy.ServiceProxy('/zlaser_viz/projection_start', Trigger)
        self.viz_stop_proj     = rospy.ServiceProxy('/zlaser_viz/projection_stop', Trigger)
        self.viz_manual_cs     = rospy.ServiceProxy('/zlaser_viz/define_coordinate_system', CoordinateSystem)
        self.viz_set_cs        = rospy.ServiceProxy('/zlaser_viz/set_coordinate_system', CoordinateSystemName)
        self.viz_show_cs       = rospy.ServiceProxy('/zlaser_viz/show_active_coordinate_system', CoordinateSystemShow)
        self.viz_rem_cs        = rospy.ServiceProxy('/zlaser_viz/remove_coordinate_system', CoordinateSystemName)
        self.viz_hide_figure   = rospy.ServiceProxy('/zlaser_viz/hide_figure', ProjectionElement)
        self.viz_unhide_figure = rospy.ServiceProxy('/zlaser_viz/unhide_figure', ProjectionElement)
        self.viz_remove_figure = rospy.ServiceProxy('/zlaser_viz/remove_figure', ProjectionElement)
       
        self.viz_monitor_fig   = rospy.ServiceProxy('/zlaser_viz/monitor_figure', ProjectionElement)

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
            rospy.set_param('projector_connected', True)
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
            rospy.set_param('projector_connected', False)
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

            # Send info to viz
            self.viz_start_proj()

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

            # Send info to viz
            self.viz_stop_proj()

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
            self.projector.define_coordinate_system(req,False)
            
            self.set_rosparam_coordinate_system(req)

            self.projector.cs_frame_create(req)
            self.projector.cs_axes_create(req)
            message = "Coordinate system correctly defined:"
            rospy.loginfo(message)
            T = self.get_user_coordinate_system(req.name)

            # Send info to viz
            self.viz_manual_cs(req)
            
            rospy.loginfo("Projecting demonstration")
            self.projector.show_coordinate_system()
            rospy.sleep(3)
            self.projector.hide_coordinate_system()
            self.projector.show_frame()
            rospy.sleep(3)
            self.projector.hide_frame()

            return CoordinateSystemResponse(T, True, message)

        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemResponse([], False, str(e))

    def auto_search_target_cb(self,req):
        """Callback of ROS service to define a new reference system by scanning the targets automatically with the projector.

        Args:
            req (list): necessary parameters to start the automatic targets search (name, aproximate coordinates of 
            reference system points, scanning area size, ...)
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an 
            information message string
        """
        rospy.loginfo("Received request to scan a new reference system.")

        try:
            self.projector.define_coordinate_system(req,True)

            coordinate_system_scanned = self.projector.get_coordinate_system_params(req.name)
            self.set_rosparam_coordinate_system(coordinate_system_scanned)

            self.projector.cs_frame_create(coordinate_system_scanned)
            self.projector.cs_axes_create(coordinate_system_scanned)
            message = "Coordinate system correctly scanned:"
            rospy.loginfo(message)
            T = self.get_user_coordinate_system(coordinate_system_scanned.name)

            # Send info to viz
            cs_params = self.read_rosparam_coordinate_system()
            self.viz_manual_cs(cs_params)
            
            rospy.loginfo("Projecting demonstration")
            self.projector.show_coordinate_system()
            rospy.sleep(3)
            self.projector.hide_coordinate_system()
            self.projector.show_frame()
            rospy.sleep(3)
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
            cs_list,active_cs = self.projector.get_coordinate_systems_list()
            return CoordinateSystemListResponse(True,"Coordinate system list:",cs_list,active_cs)

        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemListResponse(False,str(e),cs_list,active_cs)

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
            self.set_rosparam_coordinate_system(coordinate_system_params)

            # Send info to viz
            self.viz_set_cs(req)
            
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
            # Send info to viz
            self.viz_show_cs(req)

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
        """Callback of ROS service to remove a coordinate system.

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
            active_cs = self.projector.remove_coordinate_system(req.name)

            if active_cs:
                rospy.set_param('coordinate_system_name', "")

            # Send info to viz
            self.viz_rem_cs(req)

            return CoordinateSystemNameResponse(True,"Coordinate system removed")
        
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

        if not req.figure_type or not req.projection_group or not req.figure_name:
            return ProjectionElementResponse(False,"figure_type or group_name or figure_name request is empty")

        try:        
            self.projector.hide_proj_elem(req)

            # Send info to viz
            self.viz_hide_figure(req)

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

        if not req.figure_type or not req.projection_group or not req.figure_name:
            return ProjectionElementResponse(False,"figure_type or group_name or figure_name request is empty")

        try:        
            self.projector.unhide_proj_elem(req)

            # Send info to viz
            self.viz_unhide_figure(req)

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

        if not req.figure_type or not req.projection_group or not req.figure_name:
            return ProjectionElementResponse(False,"figure_type or group_name or figure_name request is empty")

        try:        
            self.projector.remove_proj_elem(req)

            # Send info to viz
            self.viz_remove_figure(req)

            return ProjectionElementResponse(True,"Figure removed")

        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(False,str(e))

    def keyboard_monitor_figure_cb(self,req):
        """Callback of ROS service to apply on real time different transformation operations (translation, rotation, scalation) to a 
        specific figure by the use of keyboard.

        Args:
            req (object): figure indentifiers
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an 
            information message string
        """
        rospy.loginfo("Received request to monitor figure. PRESS PRESS 'ESC' TO FINISH MONITORING.")

        if not req.figure_type or not req.projection_group or not req.figure_name:
            return ProjectionElementResponse(False,"figure_type or group_name or figure_name request is empty")
        
        try:
            # Send info to viz
            resp = self.viz_monitor_fig(req)
            print(resp)

            self.projector.monitor_proj_elem(req)
            rospy.loginfo("Monitoring ENDED.")
            return ProjectionElementResponse(True,"Figure monitored")
        
        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(False,str(e))

    def set_rosparam_coordinate_system(self,cs_params):
        """Set rosparams from given coordinate system.

        Args:
            cs_params (object): object with the necessary info to define a new coordinate system
        """
        rospy.set_param('coordinate_system_name', cs_params.name)
        rospy.set_param('coordinate_system_distance', cs_params.distance)
        rospy.set_param('P1/x', cs_params.P1.x)
        rospy.set_param('P1/y', cs_params.P1.y)
        rospy.set_param('P2/x', cs_params.P2.x)
        rospy.set_param('P2/y', cs_params.P2.y)
        rospy.set_param('P3/x', cs_params.P3.x)
        rospy.set_param('P3/y', cs_params.P3.y)
        rospy.set_param('P4/x', cs_params.P4.x)
        rospy.set_param('P4/y', cs_params.P4.y)
        rospy.set_param('T1/x', cs_params.T1.x)
        rospy.set_param('T1/y', cs_params.T1.y)
        rospy.set_param('coordinate_system_resolution', cs_params.resolution)
        
    def read_rosparam_coordinate_system(self):
        """Read necessary info to define a new coordinate system from rosparams.

        Returns:
            cs_params (object): object with the necessary info to define a new coordinate system
        """
        cs_params            = CoordinateSystemRequest()
        cs_params.name       = rospy.get_param('coordinate_system_name', "default_cs")
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
        cs_params.resolution = rospy.get_param('coordinate_system_resolution', 1000)
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
            rospy.set_param('projector_connected', True)
            self.open_viz_services()
            return False
        
        except Exception as e:
            rospy.logerr(e)
            return True

    def initialize_coordinate_system(self):
        """Initialize the factory or user predefined coordinate system at ros projection_node launch."""
        cs_params = self.read_rosparam_coordinate_system()
        try:
            self.projector.define_coordinate_system(cs_params,False)
            self.projector.cs_frame_create(cs_params)
            self.projector.cs_axes_create(cs_params)
            rospy.loginfo("Coordinate System [{}] loaded".format(cs_params.name))
            self.get_user_coordinate_system(cs_params.name)

            # Send info to viz
            self.viz_manual_cs(cs_params)

            rospy.loginfo("Projecting demonstration")
            self.projector.show_coordinate_system()
            rospy.sleep(0.1)
            self.projector.hide_coordinate_system()
            self.projector.show_frame()
            rospy.sleep(0.1)
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
        try:
            status = self.projector.get_connection_status()
            if status:
                rospy.loginfo("Disconnecting before shutdown...")
                self.projector.deactivate()
                self.projector.client_server_disconnect()
                rospy.loginfo("Projector disconnected.")
                return TriggerResponse(True, "Projector disconnected.")

        except Exception as e:
            rospy.logerr(e)
            return TriggerResponse(False, str(e))