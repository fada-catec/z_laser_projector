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

"""ROS node class which provides a collection of services that allows to operate and control the ZLP1 laser projector 
and simplify the task of developing further advanced features."""

import rospy
import rospkg

import math

from z_laser_zlp1.zlp_projector_manager import ZLPProjectorManager
from z_laser_zlp1.zlp_utils import  CoordinateSystemParameters

from geometry_msgs.msg import Point
from std_srvs.srv import Trigger, TriggerResponse
from z_laser_msgs.msg import Figure
from z_laser_msgs.srv import CoordinateSystem, CoordinateSystemRequest, CoordinateSystemResponse
from z_laser_msgs.srv import CoordinateSystemName, CoordinateSystemNameResponse
from z_laser_msgs.srv import CoordinateSystemShow, CoordinateSystemShowResponse
from z_laser_msgs.srv import CoordinateSystemList, CoordinateSystemListResponse
from z_laser_msgs.srv import ProjectionElement, ProjectionElementResponse

class ZLPProjectorROS(object):
    """This class initilizes the services and implements the functionalities of the node.

    Args:
        projector_IP (str): IP number of projector device
        server_IP (str): IP number of service running at projector device
        connection_port (int): connection port number 
        license_path (str): path of license file

    Attributes:
        lic_path (str): folder path where Z-Laser license file is located
        projector (object): ZLPProjectorManager object from projector_manager library
        run_viz (bool): true if visualizer is running, false otherwise
        STD_WAIT_TIME (int): predefined number of projection seconds in reference system definition
    """
    def __init__(self, projector_IP, server_IP, connection_port, license_path):
        """Initialize the ZLPProjectorROS object."""

        self.lic_path = license_path

        self.projector = ZLPProjectorManager(projector_IP, server_IP, connection_port, self.lic_path)

        self.run_viz = False

        self.STD_WAIT_TIME = CoordinateSystemParameters().DEFAULT_SHOW_TIME

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

        self.hide_proj_elem   = rospy.Service('hide_projection_element', ProjectionElement, self.hide_proj_elem_cb)
        self.unhide_proj_elem = rospy.Service('unhide_projection_element', ProjectionElement, self.unhide_proj_elem_cb)
        self.remove_proj_elem = rospy.Service('remove_projection_element', ProjectionElement, self.remove_proj_elem_cb)
        self.monit_proj_elem  = rospy.Service('monitor_projection_element', ProjectionElement, self.keyboard_monitor_proj_elem_cb)
        
        self.add_proj_elem   = rospy.Subscriber("add_projection_element", Figure, self.add_fig_cb)

        rospy.loginfo("Use ROS Services: \n\t\t\trosservice list")

    def set_viz_run(self, run):
        """Set status of run_viz attribute and define ROS publisher if required.

        Args:
            run (bool): true if visulizer is running, false otherwise
        """
        if run:
            self.run_viz = True
            self.viz_monitor_fig = rospy.Publisher('/zlaser_viz/monitor_projection_element', Figure, queue_size=10)

    def connection_cb(self, req):
        """Callback of ROS service to connect to ZLP-Server, transfer license and activate projector.

        Args:
            req (object): trigger request ROS service object 

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple 
            is an information message string
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

    def disconnection_cb(self, req):
        """Callback of ROS service to deactivate projector and disconnect from ZLP-Server.

        Args:
            req (object): trigger request ROS service object 

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple 
            is an information message string
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

    def projection_start_cb(self, req):
        """Callback of ROS service to start projection of elements related to the active reference system on the surface.
        (see :func:`set_coord_sys_cb`)

        Args:
            req (object): trigger request ROS service object 

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple 
            is an information message string
        """
        rospy.loginfo("Received request to start projection")
        try:
            self.projector.start_projection()

            # Send info to viz
            if self.run_viz:
                rospy.wait_for_service("/zlaser_viz/projection_start")
                viz_start_proj = rospy.ServiceProxy('/zlaser_viz/projection_start', Trigger)
                viz_start_proj()

            return TriggerResponse(True, "Projection started.")

        except Exception as e:
            rospy.logerr(e)
            return TriggerResponse(False, str(e))
    
    def projection_stop_cb(self, req):
        """Callback of ROS service to stop projection of all elements.

        Args:
            req (object): trigger request ROS service object 

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple 
            is an information message string
        """
        rospy.loginfo("Received request to stop projection")
        try:
            self.projector.stop_projection()

            # Send info to viz
            if self.run_viz:
                rospy.wait_for_service("/zlaser_viz/projection_stop")
                viz_stop_proj = rospy.ServiceProxy('/zlaser_viz/projection_stop', Trigger)
                viz_stop_proj()

            return TriggerResponse(True, "Projection stopped.")

        except Exception as e:
            rospy.logerr(e)
            return TriggerResponse(False, str(e))

    def manual_define_coord_sys_cb(self, req):
        """Callback of ROS service to define a new reference system, stating the points coordinates manually by the user.

        Args:
            req (object): object with the necessary parameters to define a new coordinate system
            
        Returns:
            tuple[list, bool, str]: the first value in the returned tuple is a list of the user reference points T0, T1, T2, T3,
            the second is a bool success value and the third s an information message string
        """
        rospy.loginfo("Received request to create a new coordinate system manually. Please wait for the system to indicate the end.")
        
        try:
            params = CoordinateSystemParameters.req_to_param(req)
            self.projector.define_coordinate_system(params, False)
            
            self.set_rosparam_coordinate_system(params)

            cs = self.projector.get_coordinate_system_params(params.name)
            self.projector.cs_frame_create(cs)
            self.projector.cs_axes_create(cs)
            message = "Coordinate system correctly defined:"
            rospy.loginfo(message)

            # Send info to viz
            if self.run_viz:
                rospy.wait_for_service("/zlaser_viz/define_coordinate_system")
                viz_manual_cs = rospy.ServiceProxy('/zlaser_viz/define_coordinate_system', CoordinateSystem)
                viz_manual_cs(req)
            
            rospy.loginfo("Projecting demonstration")
            self.projector.show_coordinate_system()
            rospy.sleep(self.STD_WAIT_TIME)
            self.projector.hide_coordinate_system()
            self.projector.show_frame()
            rospy.sleep(self.STD_WAIT_TIME)
            self.projector.hide_frame()

            return CoordinateSystemResponse(cs.T, True, message)

        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemResponse([], False, str(e))

    def auto_search_target_cb(self,req):
        """Callback of ROS service to define a new reference system by scanning the targets automatically with the projector.

        Args:
            req (object): object with the necessary parameters to define a new coordinate system by scanning targets
            
        Returns:
            tuple[list, bool, str]: the first value in the returned tuple is a list of the user reference points T0, T1, T2, T3,
            the second is a bool success value and the third s an information message string
        """
        rospy.loginfo("Received request to scan a new reference system.")

        try:
            params = CoordinateSystemParameters.req_to_param(req)
            self.projector.define_coordinate_system(params,True)

            coordinate_system_scanned = self.projector.get_coordinate_system_params(params.name)
            self.set_rosparam_coordinate_system(coordinate_system_scanned)

            self.projector.cs_frame_create(coordinate_system_scanned)
            self.projector.cs_axes_create(coordinate_system_scanned)
            message = "Coordinate system correctly scanned:"
            rospy.loginfo(message)

            # Send info to viz
            if self.run_viz:
                rospy.wait_for_service("/zlaser_viz/define_coordinate_system")
                viz_manual_cs = rospy.ServiceProxy('/zlaser_viz/define_coordinate_system', CoordinateSystem)
                cs_scanned_req = CoordinateSystemParameters.param_to_req(coordinate_system_scanned)
                viz_manual_cs(cs_scanned_req)

            rospy.loginfo("Projecting demonstration")
            self.projector.show_coordinate_system()
            rospy.sleep(self.STD_WAIT_TIME)
            self.projector.hide_coordinate_system()
            self.projector.show_frame()
            rospy.sleep(self.STD_WAIT_TIME)
            self.projector.hide_frame()

            return CoordinateSystemResponse(coordinate_system_scanned.T, True, message)

        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemResponse([], False, str(e))

    def get_coord_sys_list_cb(self, req):
        """Callback of ROS service to get the list of defined reference systems.

        Args:
            req (object): ROS service CoordinateSystemList request object is empty
        
        Returns:
            tuple[bool, str, list, str]: the first value in the returned tuple is a bool success value, the second 
            is an information message string, the third is a list of the defined reference systems and the last is
            the name of the active reference system
        """
        rospy.loginfo("Received request to get the coordinate system list at projector")
        
        cs_list = []
        try:
            cs_list,active_cs = self.projector.get_coordinate_systems_list()
            return CoordinateSystemListResponse(True,"Coordinate system list:",cs_list,active_cs)

        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemListResponse(False,str(e),cs_list,active_cs)

    def set_coord_sys_cb(self, req):
        """Callback of ROS service to set the indicated reference system as 'active reference system'.
        It means that services as projection_start or show_active_coordinate_system, etc. automatically use this 
        active reference system to perform their task.
        The rest of coordinate systems are defined and stored in the projector, staying on background until
        any is set as active again.

        Args:
            req (object): object with the necessary parameters to identify a coordinate system
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple 
            is an information message string
        """
        rospy.loginfo("Received request to set coordinate system.")
        if not req.name:
            return CoordinateSystemNameResponse(False,"Please, specify name")

        try:
            self.projector.set_coordinate_system(req.name)

            cs = self.projector.get_coordinate_system_params(req.name)
            self.set_rosparam_coordinate_system(cs)

            # Send info to viz
            if self.run_viz:
                rospy.wait_for_service("/zlaser_viz/set_coordinate_system")
                viz_set_cs = rospy.ServiceProxy('/zlaser_viz/set_coordinate_system', CoordinateSystemName)
                viz_set_cs(req)
            
            return CoordinateSystemNameResponse(True,"Set coordinate system")
                    
        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemNameResponse(False,str(e))

    def show_coord_sys_cb(self,req):
        """Callback of ROS service to project reference points, origin axes and frame of the active reference system.

        Args:
            req (object): object with the necessary parameters to identify a reference system
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple 
            is an information message string
        """
        rospy.loginfo("Request to project active coordinate system.")
        if req.secs == 0:
            return CoordinateSystemShowResponse(False,"Please, specify seconds")

        try:
            # Send info to viz
            if self.run_viz:
                rospy.wait_for_service("/zlaser_viz/show_active_coordinate_system")
                viz_show_cs = rospy.ServiceProxy('/zlaser_viz/show_active_coordinate_system', CoordinateSystemShow)
                viz_show_cs(req)

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
        """Callback of ROS service to remove a reference system.

        Args:
            req (object): object with the necessary parameters to identify a reference system
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple 
            is an information message string
        """
        rospy.loginfo("Received request to remove coordinate system")
        if not req.name:
            return CoordinateSystemNameResponse(False,"Please, specify name")
        
        try:
            active_cs = self.projector.remove_coordinate_system(req.name)

            if active_cs:
                self.set_rosparam_coordinate_system(CoordinateSystemParameters())

            # Send info to viz
            if self.run_viz:
                rospy.wait_for_service("/zlaser_viz/remove_coordinate_system")
                viz_rem_cs = rospy.ServiceProxy('/zlaser_viz/remove_coordinate_system', CoordinateSystemName)
                viz_rem_cs(req)

            return CoordinateSystemNameResponse(True,"Coordinate system removed")
        
        except Exception as e:
            rospy.logerr(e)
            return CoordinateSystemNameResponse(False,str(e))

    def add_fig_cb(self,msg):
        """Callback of ROS topic to define a new projection element.

        Args:
            msg (object): object with the necessary parameters to define a new projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple 
            is an information message string
        """
        rospy.loginfo("Received request to add a new projection element to the active coordinate system.")

        if not msg.projection_group or not msg.figure_name:
            return ProjectionElementResponse(False,"projection_group or figure_name request is empty.")

        try:
            if msg.figure_type == Figure.POLYLINE:
                self.projector.create_polyline(msg)
                rospy.loginfo("Line added correctly.")
            elif Figure.CIRCLE <= msg.figure_type <= Figure.OVAL:
                self.projector.create_curve(msg)
                rospy.loginfo("Curve added correctly.")
            elif msg.figure_type == Figure.TEXT:
                self.projector.create_text(msg)
                rospy.loginfo("Text added correctly.")
            else:
                return ProjectionElementResponse(False,"Figure type does not exist.")

            return ProjectionElementResponse(True,"Figure added correctly.")
        
        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(False,str(e))

    def hide_proj_elem_cb(self,req):
        """Callback of ROS service to hide specific projection element from active reference system.

        Args:
            req (object): object with the necessary parameters to identify a projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple 
            is an information message string
        """
        rospy.loginfo("Received request to hide figure.")

        if not req.projection_group or not req.figure_name:
            return ProjectionElementResponse(False,"group_name or figure_name request is empty")

        try:        
            self.projector.hide_proj_elem(req)

            # Send info to viz
            if self.run_viz:
                rospy.wait_for_service("/zlaser_viz/hide_projection_element")
                viz_hide_figure = rospy.ServiceProxy('/zlaser_viz/hide_projection_element', ProjectionElement)
                viz_hide_figure(req)

            return ProjectionElementResponse(True,"Figure hidden")

        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(False,str(e))

    def unhide_proj_elem_cb(self,req):
        """Callback of ROS service to unhide specific projection element from active reference system.

        Args:
            req (object): object with the necessary parameters to identify a projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple 
            is an information message string
        """
        rospy.loginfo("Received request to unhide figure.")

        if not req.projection_group or not req.figure_name:
            return ProjectionElementResponse(False,"group_name or figure_name request is empty")

        try:        
            self.projector.unhide_proj_elem(req)

            # Send info to viz
            if self.run_viz:
                rospy.wait_for_service("/zlaser_viz/unhide_projection_element")
                viz_unhide_figure = rospy.ServiceProxy('/zlaser_viz/unhide_projection_element', ProjectionElement)
                viz_unhide_figure(req)

            return ProjectionElementResponse(True,"Figure unhidden")

        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(False,str(e))

    def remove_proj_elem_cb(self,req):
        """Callback of ROS service to remove specific figure from active reference system.

        Args:
            req (object): object with the necessary parameters to identify a projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple 
            is an information message string
        """
        rospy.loginfo("Received request to remove figure.")

        if not req.projection_group or not req.figure_name:
            return ProjectionElementResponse(False,"group_name or figure_name request is empty")

        try:        
            self.projector.remove_proj_elem(req)

            # Send info to viz
            if self.run_viz:
                rospy.wait_for_service("/zlaser_viz/remove_projection_element")
                viz_remove_figure = rospy.ServiceProxy('/zlaser_viz/remove_projection_element', ProjectionElement)
                viz_remove_figure(req)

            return ProjectionElementResponse(True,"Figure removed")

        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(False,str(e))

    def keyboard_monitor_proj_elem_cb(self, req):
        """Callback of ROS service to apply different transformations (translation, rotation, scalation) 
        to a specific projection element on real time by the use of keyboard.

        Args:
            req (object): object with the necessary parameters to identify a projection element
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple 
            is an information message string
        """
        rospy.loginfo("Received request to monitor figure. PRESS PRESS 'ESC' TO FINISH MONITORING.")

        if not req.projection_group or not req.figure_name:
            return ProjectionElementResponse(False,"group_name or figure_name request is empty")
        
        try:
            # Send info to viz
            if self.run_viz:
                proj_elem = self.projector.get_proj_elem(req)
                figure = proj_elem.to_figure()
                
                if self.viz_monitor_fig.get_num_connections() < 1:
                    return ProjectionElementResponse(False,"No subscribers")

                self.viz_monitor_fig.publish(figure)

            self.projector.monitor_proj_elem(req)
            rospy.loginfo("Monitoring ENDED.")
            return ProjectionElementResponse(True,"Figure monitored")
        
        except Exception as e:
            rospy.logerr(e)
            return ProjectionElementResponse(False,str(e))

    def set_rosparam_coordinate_system(self, cs_params):
        """Set rosparams from given reference system.

        Args:
            cs_params (object): object with the parameters of a reference system
        """
        rospy.set_param('coordinate_system_name', cs_params.name)
        rospy.set_param('coordinate_system_distance', cs_params.distance)
        for i in range(4):
            rospy.set_param('P%d/x' % i, cs_params.P[i].x)
            rospy.set_param('P%d/y' % i, cs_params.P[i].y)
        rospy.set_param('T0/x', cs_params.T[0].x)
        rospy.set_param('T0/y', cs_params.T[0].y)
        rospy.set_param('coordinate_system_resolution', cs_params.resolution)
        
    def read_rosparam_coordinate_system(self):
        """Get parameters of the active reference system from rosparams.

        Returns:
            cs_params (object): object with the parameters of the active reference system
        """
        cs_params            = CoordinateSystemParameters()
        cs_params.name       = rospy.get_param('coordinate_system_name', "default_cs")
        cs_params.distance   = rospy.get_param('coordinate_system_distance', 1500)
        cs_params.P[0].x     = rospy.get_param('P0/x', -100)
        cs_params.P[0].y     = rospy.get_param('P0/y', -100)
        cs_params.P[1].x     = rospy.get_param('P1/x', -100)
        cs_params.P[1].y     = rospy.get_param('P1/y',  100)
        cs_params.P[2].x     = rospy.get_param('P2/x',  100)
        cs_params.P[2].y     = rospy.get_param('P2/y',  100)
        cs_params.P[3].x     = rospy.get_param('P3/x',  100)
        cs_params.P[3].y     = rospy.get_param('P3/y', -100)
        cs_params.T[0].x     = rospy.get_param('T0/x',    0)
        cs_params.T[0].y     = rospy.get_param('T0/y',    0)
        cs_params.resolution = rospy.get_param('coordinate_system_resolution', 1000)
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
            rospy.set_param('projector_connected', True)
            return False
        
        except Exception as e:
            rospy.logerr(e)
            return True

    def initialize_coordinate_system(self):
        """Initial projector setup with the factory or an user predefined reference system from configuration files."""
        params = self.read_rosparam_coordinate_system()
        try:
            self.projector.define_coordinate_system(params, False)
            
            cs = self.projector.get_coordinate_system_params(params.name)
            self.projector.cs_frame_create(cs)
            self.projector.cs_axes_create(cs)
            rospy.loginfo("Coordinate System [{}] loaded".format(params.name))

            # Send info to viz
            if self.run_viz:
                rospy.wait_for_service("/zlaser_viz/define_coordinate_system")
                viz_manual_cs = rospy.ServiceProxy('/zlaser_viz/define_coordinate_system', CoordinateSystem)
                cs_req = CoordinateSystemParameters.param_to_req(params)
                viz_manual_cs(cs_req)

            rospy.loginfo("Projecting demonstration")
            self.projector.show_coordinate_system()
            rospy.sleep(self.STD_WAIT_TIME)
            self.projector.hide_coordinate_system()
            self.projector.show_frame()
            rospy.sleep(self.STD_WAIT_TIME)
            self.projector.hide_frame()

        except Exception as e:
            if "InvalidRelativePath" in str(e):
                rospy.logwarn("Possible error: check coordinate system name is not empty")
            rospy.logerr(e)

    def shutdown_handler(self):
        """Handler to close connection when node exits.
 
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple 
            is an information message string
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