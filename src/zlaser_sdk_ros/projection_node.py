#!/usr/bin/env python3

import rospy
import rospkg
import os
import time
import numpy as np

from zlaser_sdk_ros.projector_manager import ProjectorManager
from zlaser_sdk_ros.utils import CoordinateSystemParameters, ProjectionElementParameters

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
from zlaser_sdk_ros.srv import CsRefPoints, CsRefPointsResponse, CoordinateSystem, CoordinateSystemResponse
from zlaser_sdk_ros.srv import ProjectionElement, ProjectionElementResponse

class ProjectionNode:
    def __init__(self):

        self.node_name = 'projection_node'
        rospy.init_node(self.node_name)
        rospy.loginfo("MAIN")
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('zlaser_sdk_ros')

        self.lic_path = self.pkg_path + "/lic/1900027652.lic"
        
        projector_IP = "192.168.10.10"
        server_IP = "192.168.10.11"
        connection_port = 9090
        self.projector = ProjectorManager(projector_IP, server_IP, connection_port)

        self.connect_srv  = rospy.Service('/projector_srv/connect', Trigger, self.connection_cb)
        self.discnt_srv   = rospy.Service('/projector_srv/disconnect', Trigger, self.disconnection_cb)
        self.lic_srv      = rospy.Service('/projector_srv/load_license', Trigger, self.transfer_license_cb)
        self.setup_srv    = rospy.Service('/projector_srv/setup', Trigger, self.setup_cb)
        self.start_srv    = rospy.Service('/projector_srv/projection_start', Trigger, self.projection_start_cb)
        self.stop_srv     = rospy.Service('/projector_srv/projection_stop', Trigger, self.projection_stop_cb)

        self.def_cs_srv   = rospy.Service('/projector_srv/man_def_cs', CsRefPoints, self.manual_define_coord_sys_cb)

        self.get_cs_list  = rospy.Service('/projector_srv/cs_list', CoordinateSystem, self.get_coord_sys_list_cb)
        self.set_cs       = rospy.Service('/projector_srv/set_cs', CoordinateSystem, self.set_coord_sys_cb)
        self.show_srv     = rospy.Service('/projector_srv/show_current_cs', CoordinateSystem, self.show_coord_sys_cb)
        self.rem_cs       = rospy.Service('/projector_srv/remove_coord_sys', CoordinateSystem, self.remove_coord_sys_cb)

        self.shape_srv    = rospy.Service('/projector_srv/add_shape', ProjectionElement, self.add_shape_cb)
        self.hd_shp_srv   = rospy.Service('/projector_srv/hide_shape', ProjectionElement, self.hide_shape_cb)
        self.unhd_shp_srv = rospy.Service('/projector_srv/unhide_shape', ProjectionElement, self.unhide_shape_cb)
        self.rem_shp_srv  = rospy.Service('/projector_srv/remove_shape', ProjectionElement, self.remove_shape_cb)
        
        rospy.spin()

    def connection_cb(self,req):
        rospy.loginfo("Received request to connect and activate projector.")

        s,m = self.projector.client_server_connect()
        if not s:
            rospy.logerr(m)
            return TriggerResponse(s,m)
        
        s,m = self.projector.activate()
        if not s:
            rospy.logerr(m)
            return TriggerResponse(s,m)
        
        m = "Projector connected and activated."
        rospy.loginfo(m)
        return TriggerResponse(s,m)

    def disconnection_cb(self,req):
        rospy.loginfo("Received request to disconnect projector.")
        
        s,m = self.projector.deactivate()
        if not s:
            rospy.logerr(m)
            return TriggerResponse(s,m)
        
        s,m = self.projector.client_server_disconnect()
        if not s:
            rospy.logerr(m)
            return TriggerResponse(s,m)

        m = "Projector deactivated and disconnected."
        rospy.loginfo(m)
        return TriggerResponse(s,m)

    def transfer_license_cb(self,req):
        rospy.loginfo("Received request to load license file.")

        s,m = self.projector.load_license(self.lic_path)
        if not s: 
            rospy.logerr(m)
            rospy.logwarn("Load license again: \n\n rosservice call /projector_srv/load_license")
            return TriggerResponse(s,m)
        
        s,m = self.projector.geotree_operator_create()
        if not s:
            rospy.logerr(m)
            return TriggerResponse(s,m)
        
        m = "License correctly loaded."
        rospy.loginfo(m)
        return TriggerResponse(s,m)

    def setup_cb(self,req):
        rospy.loginfo("Received request to setup projector")

        s,m = self.projector.client_server_connect()
        if not s:
            rospy.logerr(m)
            return TriggerResponse(s,m)

        s,m = self.projector.activate()
        if not s:
            rospy.logerr(m)
            return TriggerResponse(s,m)

        rospy.loginfo(m)
        s,m = self.projector.load_license(self.lic_path)
        if not s: 
            rospy.logerr(m)
            rospy.logwarn("Load license again: \n\n rosservice call /projector_srv/setup")
            return TriggerResponse(s,m)

        rospy.loginfo(m)
        s,m = self.projector.geotree_operator_create()
        if not s:
            rospy.logerr(m)
            return TriggerResponse(s,m)
        
        m = "Projector setup correct: activated, connected and license loaded."
        rospy.loginfo(m)

        return TriggerResponse(s,m)

    def projection_start_cb(self,req):
        rospy.loginfo("Received request to start projection")
        
        s,m = self.projector.start_projection()
        if s:
            rospy.loginfo(m)
        else:
            rospy.logerr(m)
        
        return TriggerResponse(s,m)
    
    def projection_stop_cb(self,req):
        rospy.loginfo("Received request to stop projection")
        
        s,m = self.projector.stop_projection()
        if s:
            rospy.loginfo(m)
        else:
            rospy.logerr(m)
        
        return TriggerResponse(s,m)

    def manual_define_coord_sys_cb(self,req):
        rospy.loginfo("Received request to create a new coordinate system manually. Please wait for the system to indicate the end.")
        
        cs_params = CoordinateSystemParameters()
        cs_params.set_request_params(req)

        s,m = self.projector.define_coordinate_system(cs_params)
        if not s:
            rospy.logerr(m)
            return CsRefPointsResponse(Bool(s),String(m))
        
        s,m = self.projector.show_coordinate_system(5)
        if not s:
            rospy.logerr(m)
            return CsRefPointsResponse(Bool(s),String(m))
        
        s,m = self.projector.cs_axes_create(cs_params)
        if not s:
            rospy.logerr(m)
            return CsRefPointsResponse(Bool(s),String(m))
            
        m = "Coordinate system correctly defined"
        rospy.loginfo(m)
        return CsRefPointsResponse(Bool(s),String(m))

    def get_coord_sys_list_cb(self,req):
        rospy.loginfo("Received request to get the coordinate system list at projector")
        
        cs_list = []
        cs_list = [String(cs_name) for cs_name in cs_list]
        
        if not req.get_list.data:
            s = False
            m = "get_list request not True"
            return CoordinateSystemResponse(Bool(s),String(m),cs_list)

        cs_list,s,m = self.projector.get_coordinate_systems()
        if not s:
                rospy.logerr(m)
                return CoordinateSystemResponse(Bool(s),String(m),cs_list)

        cs_list = [String(cs_name) for cs_name in cs_list]
        return CoordinateSystemResponse(Bool(s),String(m),cs_list)

    def set_coord_sys_cb(self,req):
        rospy.loginfo("Received request to set coordinate system.")

        cs_list = []
        cs_list = [String(cs_name) for cs_name in cs_list]

        if not req.set.data or not req.cs_name.data:
            s = False
            m = "set request is not True or cs_name request is empty"
            return CoordinateSystemResponse(Bool(s),String(m),cs_list)

        s,m = self.projector.set_coordinate_system(req.cs_name.data)
        if not s:
            rospy.logerr(m)
            return CoordinateSystemResponse(Bool(s),String(m),cs_list)

        return CoordinateSystemResponse(Bool(s),String(m),cs_list)

    def show_coord_sys_cb(self,req):
        rospy.loginfo("Request to project current coordinate system.")

        cs_list = []
        cs_list = [String(cs_name) for cs_name in cs_list]

        if not req.show_current.data or not req.secs.data:
            s = False
            m = "show_current request not True or secs request is empty"
            return CoordinateSystemResponse(Bool(s),String(m),cs_list)

        s,m = self.projector.show_coordinate_system(req.secs.data)
        if not s:
            rospy.logerr(m)
            return CoordinateSystemResponse(Bool(s),String(m),cs_list)
   
        s,m = self.projector.cs_axes_unhide()
        if not s:
            rospy.logerr(m)
            return CoordinateSystemResponse(Bool(s),String(m),cs_list)

        m = "Coordinate system showed correctly."
        rospy.loginfo(m)
        return CoordinateSystemResponse(Bool(s),String(m),cs_list)

    def remove_coord_sys_cb(self,req):
        rospy.loginfo("Received request to remove coordinate system")
        
        cs_list = []
        cs_list = [String(cs_name) for cs_name in cs_list]

        if not req.remove.data or not req.cs_name.data:
            s = False
            m = "set request not True or cs_name request is empty"
            return CoordinateSystemResponse(Bool(s),String(m),cs_list)
        
        s,m = self.projector.remove_coordinate_system(req.cs_name.data)
        if not s:
            rospy.logerr(m)
            return CoordinateSystemResponse(Bool(s),String(m),cs_list)
        
        return CoordinateSystemResponse(Bool(s),String(m),cs_list)

    def add_shape_cb(self,req):
        rospy.loginfo("Received request to add a shape to the current coordinate system.")

        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.set_request_params(req)
        
        if not req.add.data or not (req.shape_type.data and req.projection_group_name.data and req.shape_id.data):
            s = False
            m = "add request not True or shape_type or projection_group_name or shape_id request is empty."
            return ProjectionElementResponse(Bool(s),String(m))

        if req.shape_type.data == "polyline":
            rospy.loginfo("Creating polyline shape.")
            s,m = self.projector.create_polyline(proj_elem_params)
            if not s:
                rospy.logerr(m)
                return ProjectionElementResponse(Bool(s),String(m))
            m = "Polyline added correctly."
            rospy.loginfo(m)
        else: 
            s = False
            m = "shape_type does not match any category."

        return ProjectionElementResponse(Bool(s),String(m))
    
    def hide_shape_cb(self,req):
        rospy.loginfo("Received request to hide shape.")

        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.set_request_params(req)

        if not req.hide.data or not req.shape_type.data or not req.projection_group_name.data or not req.shape_id.data:
            s = False
            m = "hide request not True or shape_type or projection_group_name or shape_id request is empty"
            return ProjectionElementResponse(Bool(s),String(m))
        
        s,m = self.projector.hide_shape(proj_elem_params)
        if not s:
            rospy.logerr(m)
            return ProjectionElementResponse(Bool(s),String(m))
            
        return ProjectionElementResponse(Bool(s),String(m))

    def unhide_shape_cb(self,req):
        rospy.loginfo("Received request to unhide shape.")

        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.set_request_params(req)

        if not req.unhide.data or not req.shape_type.data or not req.projection_group_name.data or not req.shape_id.data:
            s = False
            m = "unhide request not True or shape_type or projection_group_name or shape_id request is empty"
            return ProjectionElementResponse(Bool(s),String(m))

        s,m = self.projector.unhide_shape(proj_elem_params)
        if not s:
            rospy.logerr(m)
            return ProjectionElementResponse(Bool(s),String(m))

        return ProjectionElementResponse(Bool(s),String(m))

    def remove_shape_cb(self,req):
        rospy.loginfo("Received request to remove shape.")

        proj_elem_params = ProjectionElementParameters()
        proj_elem_params.set_request_params(req)

        if not req.remove.data or not req.shape_type.data or not req.projection_group_name.data or not req.shape_id.data:
            s = False
            m = "remove request not True or shape_type or projection_group_name or shape_id request is empty"
            return ProjectionElementResponse(Bool(s),String(m))
        
        s,m = self.projector.remove_shape(proj_elem_params)
        if not s:
            rospy.logerr(m)
            return ProjectionElementResponse(Bool(s),String(m))

        return ProjectionElementResponse(Bool(s),String(m))

def main():
    ProjectionNode()

if __name__ == '__main__':
    main()