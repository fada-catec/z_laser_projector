#!/usr/bin/env python3

import rospy
import rospkg
import os
import time
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool, String
from projector_manager import ProjectorManager
from zlp import CoordinateSystemParameters, ProjectionElementParameters

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
        self.coordinate_system = ""

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
        if s:
            rospy.loginfo(m)
            s,m = self.projector.activate()
            if s:
                rospy.loginfo(m)
                m = "Projector connected and activated."
            else:
                rospy.logerr(m)
        else:
            rospy.logerr(m)

        return TriggerResponse(s,m)

    def disconnection_cb(self,req):
        rospy.loginfo("Received request to disconnect projector.")
        
        s,m = self.projector.deactivate()
        if s:
            rospy.loginfo(m)
            s,m = self.projector.client_server_disconnect()
            if s:
                rospy.loginfo(m)
                m = "Projector deactivated and disconnected."
            else:
                rospy.logerr(m)
        else:
            rospy.logerr(m)

        return TriggerResponse(s,m)

    def transfer_license_cb(self,req):
        rospy.loginfo("Received request to load license file.")

        s,m1,m2 = self.projector.load_license(self.lic_path)
        if s: 
            rospy.loginfo(m1)
            s,m = self.projector.geotree_operator_create()
            if s:
                rospy.loginfo(m)
                m = "License correctly loaded."
            else:
                rospy.logerr(m)
        else:
            rospy.logerr(m2)
            m = m2
            rospy.logwarn("Load license again: \n\n rosservice call /projector_srv/load_license") 

        return TriggerResponse(s,m)

    def setup_cb(self,req):
        rospy.loginfo("Received request to setup projector")

        s,m = self.projector.client_server_connect()
        if s:
            rospy.loginfo(m)
            s,m = self.projector.activate()
            if s:
                rospy.loginfo(m)
                s,m1,m2 = self.projector.load_license(self.lic_path)
                if s: 
                    rospy.loginfo(m1)
                    s,m = self.projector.geotree_operator_create()
                    if s:
                        rospy.loginfo(m)
                        m = "Projector activated, connected and license correctly loaded."
                    else:
                        rospy.logerr(m)
                else:
                    rospy.logerr(m2)
                    m = m2
                    rospy.logwarn("Load license again: \n\n rosservice call /projector_srv/load_license") 
            else:
                rospy.logerr(m)
        else:
            rospy.logerr(m)

        rospy.loginfo("No Current Coordinate System set so far.")

        return TriggerResponse(s,m)

    def projection_start_cb(self,req):
        rospy.loginfo("Received request to start projection")
        
        s,m = self.projector.start_projection(self.coordinate_system)
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
        
        cs_params = CoordinateSystemParameters(req)

        self.coordinate_system,s,m = self.projector.define_coordinate_system(cs_params)
        if s:
            rospy.loginfo(m)
            s,m = self.projector.show_coordinate_system(self.coordinate_system,5)
            if s:
                rospy.loginfo(m)
                s,m = self.projector.cs_axes_create(self.coordinate_system,cs_params)
                if s:
                    rospy.loginfo(m)
                else:
                    rospy.logerr(m)

            else:
                rospy.logerr(m)
        else:
            rospy.logerr(m)

        return CsRefPointsResponse(Bool(s),String(m))

    def get_coord_sys_list_cb(self,req):
        rospy.loginfo("Received request to get the coordinate system list at projector")
        
        if req.get_list.data:
            cs_list,s,m = self.projector.get_coordinate_systems()
            if s:
                rospy.loginfo(m)
            else:
                rospy.logerr(m)
        else:
            s = False
            m = "get_list request not True"
            cs_list = []

        rospy.loginfo("coordinate systems list: {}".format(cs_list))

        return CoordinateSystemResponse(Bool(s),String(m),String(cs_list))

    def set_coord_sys_cb(self,req):
        rospy.loginfo("Received request to set coordinate system.")

        if req.set.data:
            if req.cs_name.data:
                s,m = self.projector.set_coordinate_system(req.cs_name.data)
                if s:
                    rospy.loginfo(m)
                    self.coordinate_system = req.cs_name.data
                else:
                    rospy.logerr(m)
            else:
                s = False
                m = "cs_name request is empty"
        else:
            s = False
            m = "set request not True"

        return CoordinateSystemResponse(Bool(s),String(m),String([]))

    def show_coord_sys_cb(self,req):
        rospy.loginfo("Request to project current coordinate system.")

        if req.show_current.data:
            if req.secs.data:
                s,m = self.projector.show_coordinate_system(self.coordinate_system,req.secs.data)
                if s:
                    rospy.loginfo(m)
                    s,m = self.projector.cs_axes_unhide(self.coordinate_system)
                    if s:
                        rospy.loginfo(m)
                    else:
                        rospy.logerr(m)
                else:
                    rospy.logerr(m)
            else:
                s = False
                m = "secs request is empty"
        else:
            s = False
            m = "show_current request not True"

        return CoordinateSystemResponse(Bool(s),String(m),String([]))

    def remove_coord_sys_cb(self,req):
        rospy.loginfo("Received request to remove coordinate system")
        
        if req.remove.data:
            if req.cs_name.data:
                s,m = self.projector.remove_coordinate_system(req.cs_name.data)
                if s:
                    rospy.loginfo(m)
                    self.coordinate_system = ""
                else:
                    rospy.logerr(m)
            else:
                s = False
                m = "cs_name request is empty"
        else:
            s = False
            m = "set request not True"

        rospy.loginfo("Set other coordinate system or define a new one before continue")

        return CoordinateSystemResponse(Bool(s),String(m),String([]))

    def add_shape_cb(self,req):
        rospy.loginfo("Received request to add a shape to the current coordinate system.")

        proj_elem_params = ProjectionElementParameters(req)
        proj_elem_params.set_params()
        
        if req.add.data:
            if req.shape_type.data and req.projection_group_name.data and req.shape_id.data:
                if req.shape_type.data == "polyline":
                    rospy.loginfo("Creating polyline shape")
                    s,m = self.projector.create_polyline(self.coordinate_system,proj_elem_params)
                    if s:
                        rospy.loginfo(m)
                    else:
                        rospy.logerr(m)
                else: 
                    s = False
                    m = "shape_type does not match any"
            else:
                s = False
                m = "shape_type or projection_group_name or shape_id request is empty"
        else:
            s = False
            m = "add request not True"
        
        return ProjectionElementResponse(Bool(s),String(m))
    
    def hide_shape_cb(self,req):
        rospy.loginfo("Received request to hide shape.")

        proj_elem_params = ProjectionElementParameters(req) # en hide_shape req.x está vacío, hay problemas??
                                                            # hay algun problema por machacar proj_elem_params??
        proj_elem_params.set_params()

        if req.hide.data:
            if req.shape_type.data and req.projection_group_name.data and req.shape_id.data:
                s,m = self.projector.hide_shape(proj_elem_params)
                if s:
                    rospy.loginfo(m)
                else:
                    rospy.logerr(m)
            else:
                s = False
                m = "shape_type or projection_group_name or shape_id request is empty"
        else:
            s = False
            m = "add request not True"

        return ProjectionElementResponse(Bool(s),String(m))

    def unhide_shape_cb(self,req):
        rospy.loginfo("Received request to unhide shape.")

        proj_elem_params = ProjectionElementParameters(req)
        proj_elem_params.set_params()

        if req.unhide.data:
            if req.shape_type.data and req.projection_group_name.data and req.shape_id.data:
                s,m = self.projector.unhide_shape(proj_elem_params)
                if s:
                    rospy.loginfo(m)
                else:
                    rospy.logerr(m)
            else:
                s = False
                m = "shape_type or projection_group_name or shape_id request is empty"
        else:
            s = False
            m = "add request not True"

        return ProjectionElementResponse(Bool(s),String(m))

    def remove_shape_cb(self,req):
        rospy.loginfo("Received request to remove shape.")

        proj_elem_params = ProjectionElementParameters(req)
        proj_elem_params.set_params()

        if req.unhide.data:
            if req.shape_type.data and req.projection_group_name.data and req.shape_id.data:
                s,m = self.projector.remove_shape(proj_elem_params)
                if s:
                    rospy.loginfo(m)
                else:
                    rospy.logerr(m)
            else:
                s = False
                m = "shape_type or projection_group_name or shape_id request is empty"
        else:
            s = False
            m = "add request not True"

        return ProjectionElementResponse(Bool(s),String(m))


def main():
    ProjectionNode()

if __name__ == '__main__':
    main()