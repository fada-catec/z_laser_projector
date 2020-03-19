#!/usr/bin/env python3

import rospy
import rospkg
import os
import time
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool
from projector_manager import ProjectorManager
from zlaser_sdk_ros.srv import ShapeParams, ShapeParamsResponse, CsRefPoints, CsRefPointsResponse, ShowCs, ShowCsResponse, RemovCs, RemovCsResponse
#from zlaser_sdk_ros.projector_manager import ProjectorManager

class ProjectionNode:
    def __init__(self):

        self.node_name = 'projection_node'
        rospy.init_node(self.node_name)
        rospy.loginfo("MAIN")
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('zlaser_sdk_ros')
        # self.setup_path = self.pkg_path + "/scripts/set_up_projector.py" #not used
        self.lic_path = self.pkg_path + "/lic/1900027652.lic"

        self.projector = ProjectorManager() # Create projector object

        # Open services
        self.cnt_srv     = rospy.Service('/projector_srv/connect', Trigger, self.connection_cb)
        self.discnt_srv  = rospy.Service('/projector_srv/disconnect', Trigger, self.disconnection_cb)
        self.lic_srv     = rospy.Service('/projector_srv/load_license', Trigger, self.transfer_license_cb)
        self.setup_srv   = rospy.Service('/projector_srv/setup', Trigger, self.setup_cb)

        self.get_cs_list = rospy.Service('/projector_srv/cs_list', Trigger, self.get_coord_sys_list_cb)
        self.cs_srv      = rospy.Service('/projector_srv/man_def_cs', CsRefPoints, self.manual_define_coord_sys_cb)
        self.show_srv    = rospy.Service('/projector_srv/show_cs', ShowCs, self.show_coord_sys_cb)
        self.rem_cs      = rospy.Service('/projector_srv/remove_coord_sys', RemovCs, self.remove_coord_sys_cb)

        self.shape_srv   = rospy.Service('/projector_srv/add_shape', ShapeParams, self.add_shape_cb)
        self.start_srv   = rospy.Service('/projector_srv/projection_start', Trigger, self.projection_start_cb)
        self.stop_srv    = rospy.Service('/projector_srv/projection_stop', Trigger, self.projection_stop_cb)
        
        rospy.spin()

    def connection_cb(self,req):
        rospy.loginfo("Received request to start projector")
        e = self.projector.client_server_connect()
        rospy.loginfo(e)
        e = self.projector.activate()
        rospy.loginfo(e)
        return TriggerResponse(True,str(e))

    def disconnection_cb(self,req):
        rospy.loginfo("Received request to stop projector")
        e = self.projector.deactivate()
        rospy.loginfo(e)
        e = self.projector.client_server_disconnect()
        rospy.loginfo(e)
        return TriggerResponse(True,str(e))

    def transfer_license_cb(self,req):
        self.projector.license_path = self.lic_path
        e = self.projector.transfer_license()
        rospy.loginfo(e)
        e = self.projector.function_module_create()
        rospy.loginfo(e)
        return TriggerResponse(True,"License loaded and function module created")

    def setup_cb(self,req):
        rospy.loginfo("Received request to setup projector")
        e = self.projector.client_server_connect() # connect to service
        rospy.loginfo(e)
        e = self.projector.activate() # activate projector
        rospy.loginfo(e)
        self.projector.license_path = self.lic_path
        e = self.projector.transfer_license() # transfer license
        rospy.loginfo(e)
        if not self.projector.check_license(): # check license
            rospy.logwarn("License is not valid. Load a new one: \n\n rosservice call /projector_srv/load_license")      
            return TriggerResponse(False,"end setup")    
        else:
            rospy.loginfo("License is valid...")
            e = self.projector.function_module_create() # create function module 
            rospy.loginfo(e)
            cs_list = self.projector.get_coordinate_systems() # check coordinate system
            rospy.loginfo("Available coordinate systems: {}".format(cs_list))
            rospy.loginfo("Factory coordinate system: {}".format(cs_list[0]))
            
            # e = self.projector.set_coordinate_system(cs_list[0]) # set default coordinate system
            # rospy.loginfo(e)
            # self.projector.show_coordinate_system(cs_list[0],5) 
            # Not showing default cs at setup because if others cs exist at the beginning, it's going to project de last one registered, i.e., the last one
            # which has been executed within the registering function (register_coordinate_system)
            # To choose the next cs to project between the cs defined, we need to execute the registering function, and for that we need its related reference_object
            # For our own defined cs, we have the reference_objects, but we don't have the ref_obj related to the default cs

        return TriggerResponse(True,"end setup")



    # def set_coord_system(self,cs):
    #     if len(cs)>1:
    #         rospy.loginfo("Received request to set coordinate system. Setting [{}] as coordinate system".format(cs))
    #         self.projector.set_coordinate_system(cs[-1])

    def get_coord_sys_list_cb(self,req):
        rospy.loginfo("Received request to get the current coordinate system list at projector")
        cs_list = self.projector.get_coordinate_systems() # check coordinate system
        rospy.loginfo("Available coordinate systems: {}".format(cs_list))
        return TriggerResponse(True,"End list cs")

    def manual_define_coord_sys_cb(self,req):
        rospy.loginfo("Received request to create a new coordinate system manually. Please wait for the system to indicate the end")
        cs = self.projector.define_coordinate_system(req) # define coordinate system
        e = self.projector.set_coordinate_system(cs) # set new coordinate system
        rospy.loginfo(e)
        e = self.projector.register_coordinate_system(cs) # register coordinate system
        rospy.loginfo(e)
        e = self.projector.show_coordinate_system(cs,5) # show_coord_sys: name, project points, project axis, print SC properties (position, distance, etc.) # show created coordinate system for secs
        rospy.loginfo(e)
        return CsRefPointsResponse(Bool(True))

    def show_coord_sys_cb(self,req): # SHOW AND SET. show_coord_sys: name, project points, project axis, print SC properties (position, distance, etc.)
        rospy.loginfo("Request to project coordinate system: {}".format(req.cs_name.data))
        e = self.projector.set_coordinate_system(req.cs_name.data) # set default coordinate system
        rospy.loginfo(e)
        self.projector.show_coordinate_system(req.cs_name.data,req.secs.data)
        return ShowCsResponse(Bool(True))

    def remove_coord_sys_cb(self,req):
        rospy.loginfo("Received request to remove [{}] coordinate system".format(req.cs_name.data))
        e = self.projector.remove_coordinate_system(req.cs_name.data) 
        rospy.loginfo(e)
        return RemovCsResponse(Bool(True))



    def add_shape_cb(self,req):
        rospy.loginfo("Received request to project a: '{}', at [{}] coordinate system".format(req.shape_type.data, self.projector.coordinate_system))
        
        if req.shape_type.data == "polyline":
            rospy.loginfo("Creating polyline shape")
            e = self.projector.create_polyline(req.projection_group_name.data,req.shape_id.data,req.x.data,req.y.data,req.angle.data,req.r.data)
            rospy.loginfo(e)
            # rospy.loginfo(" ----- Projecting Polyline ----- ")
            # self.projector.start_projection()
        elif req.shape_type.data == "circle":
            rospy.loginfo("Creating circle shape")
            e = self.projector.create_circle(req.projection_group_name.data,req.shape_id.data,req.x.data,req.y.data,req.r.data)
            rospy.loginfo(e)
            # rospy.loginfo(" ----- Projecting Polyline ----- ") 
            # self.projector.start_projection()
        else: 
            return ShapeParamsResponse(Bool(False))
        return ShapeParamsResponse(Bool(True))

    def projection_start_cb(self,req):
        self.projector.start_projection()
        return TriggerResponse(True,"Started")
    
    def projection_stop_cb(self,req):
        self.projector.stop_projection()
        return TriggerResponse(True,"Stopped")

def main():
    ProjectionNode()

if __name__ == '__main__':
    main()