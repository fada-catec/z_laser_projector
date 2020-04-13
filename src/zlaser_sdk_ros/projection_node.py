#!/usr/bin/env python3

import rospy
import rospkg
import os
import time
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool
from projector_manager import ProjectorManager
from zlaser_sdk_ros.srv import ShapeParams, ShapeParamsResponse, CsRefPoints, CsRefPointsResponse, ShowCs, ShowCsResponse
from zlaser_sdk_ros.srv import RemovCs, RemovCsResponse, RemovShape, RemovShapeResponse, HideShape, HideShapeResponse
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
        self.coordinate_system = ""


        projector_IP = "192.168.10.10"
        server_IP = "192.168.10.11"
        connection_port = 9090
        self.projector = ProjectorManager(projector_IP, server_IP, connection_port)


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
        self.h_shape_srv = rospy.Service('/projector_srv/hide_shape', HideShape, self.hide_shape_cb)
        self.r_shape_srv = rospy.Service('/projector_srv/remove_shape', RemovShape, self.remove_shape_cb)
        
        rospy.spin()

    def connection_cb(self,req):
        rospy.loginfo("Received request to start projector")
        e = self.projector.client_server_connect()
        rospy.loginfo(e)
        print(type(e)) #<class 'thriftpy.transport.TTransportException'>
        # if e = TTransportException(type=1, message="Could not connect to ('192.168.10.11', 9090)")

        # log.info("Connecting to ZLP Service at %s:%s" % (ip, port))

        # log.info("Activating projector. ID: " + self.projector_id)
        e = self.projector.activate()
        rospy.loginfo(e)
        return TriggerResponse(True,str(e))

    def disconnection_cb(self,req):
        # HAY QUE DISTINGUIR ENTRE DESCONECTAR Y BORRAR TODO O DESCONECTAR SOLO
        rospy.loginfo("Received request to stop projector")
        
        # log.info("Deactivating projection of projector: " + self.projector_id)
        e = self.projector.deactivate()
        rospy.loginfo(e)
        e = self.projector.client_server_disconnect()
        rospy.loginfo(e)
        return TriggerResponse(True,str(e))

    def transfer_license_cb(self,req):
        self.projector.license_path = self.lic_path
        e = self.projector.load_license()
        rospy.loginfo(e)
        if not self.projector.check_license(): # check license
            rospy.logwarn("License is not valid. Load a new one: \n\n rosservice call /projector_srv/load_license")      
            return TriggerResponse(False,"end setup")    
        else:
            rospy.loginfo("License is valid...")
            e = self.projector.geotree_operator_create() # create function module 
            rospy.loginfo(e)
        return TriggerResponse(True,"License loaded and function module created")

    def setup_cb(self,req):
        rospy.loginfo("Received request to setup projector")
        e = self.projector.client_server_connect() # connect to service
        rospy.loginfo(e)
        e = self.projector.activate() # activate projector
        rospy.loginfo(e)
        self.projector.license_path = self.lic_path
        e = self.projector.load_license() # transfer license
        rospy.loginfo(e)
        if not self.projector.check_license(): # check license
            rospy.logwarn("License is not valid. Load a new one: \n\n rosservice call /projector_srv/load_license")      
            return TriggerResponse(False,"end setup")    
        else:
            rospy.loginfo("License is valid...")
            e = self.projector.geotree_operator_create() # create function module 
            rospy.loginfo(e)
            cs_list = self.projector.get_coordinate_systems() # check coordinate system
            rospy.loginfo("Available coordinate systems: {}".format(cs_list))
            rospy.loginfo("Factory coordinate system: [{}]".format(cs_list[0]))
            rospy.loginfo("CURRENT coordinate system: [{}] ".format(self.coordinate_system))
            
            # e = self.projector.set_coordinate_system(cs_list[0]) # set default coordinate system
            # rospy.loginfo(e)
            # self.projector.show_coordinate_system(cs_list[0],5) 
            # Not showing default cs at setup because if others cs exist at the beginning, it's going to project de last one registered, i.e., the last one
            # which has been executed within the registering function (register_coordinate_system)
            # To choose the next cs to project between the cs defined, we need to execute the registering function, and for that we need its related reference_object
            # For our own defined cs, we have the reference_objects, but we don't have the ref_obj related to the default cs

        return TriggerResponse(True,"end setup")

    def get_coord_sys_list_cb(self,req):
        rospy.loginfo("Received request to get the coordinate system list at projector")
        cs_list = self.projector.get_coordinate_systems() # check coordinate system
        rospy.loginfo("Available coordinate systems: {}".format(cs_list))
        return TriggerResponse(True,"end cs list")

    def manual_define_coord_sys_cb(self,req):
        
        rospy.loginfo("Received request to create a new coordinate system manually. Please wait for the system to indicate the end")
        
        self.coordinate_system = self.projector.define_coordinate_system(req) # define new coordinate system
        e = self.projector.set_coordinate_system(self.coordinate_system) # set new coordinate system
        rospy.loginfo(e)
        rospy.loginfo("CURRENT coordinate system: [{}] ".format(self.coordinate_system))
        e = self.projector.register_coordinate_system(self.coordinate_system) # register coordinate system
        rospy.loginfo(e)
        self.projector.get_coordinate_systems() # print CURRENT cs list
        e = self.projector.show_coordinate_system(self.coordinate_system,5) # show_coord_sys: name, project points, project axis, print SC properties (position, distance, etc.) # show created coordinate system for secs
        rospy.loginfo(e)
        
        e = self.projector.create_polyline(self.coordinate_system, self.coordinate_system + "_origin","axis_x",req.T1_x.data,req.T1_y.data,0,50) # origin x axis to project, angle = 0
        rospy.loginfo(e)
        e = self.projector.create_polyline(self.coordinate_system, self.coordinate_system + "_origin","axis_y",req.T1_x.data,req.T1_y.data,90,50) # origin y axis to project, angle = 90
        rospy.loginfo(e)

        e = self.projector.start_projection(self.coordinate_system)
        rospy.loginfo(e)
        input("PROJECTING COORDINATE SYSTEM ORIGIN AXES. PRESS ENTER TO FINISH.")
        e = self.projector.stop_projection()
        rospy.loginfo(e)

        e = self.projector.hide_shape(self.coordinate_system + "_origin","polyline","axis_x")
        rospy.loginfo(e)
        e = self.projector.hide_shape(self.coordinate_system + "_origin","polyline","axis_y")
        rospy.loginfo(e)
        return CsRefPointsResponse(Bool(True))

    def show_coord_sys_cb(self,req): # SHOW AND SET. show_coord_sys: name, project points, project axis, print SC properties (position, distance, etc.)
        rospy.loginfo("Request to project (and set) coordinate system: {}".format(req.cs_name.data))
        
        self.coordinate_system = req.cs_name.data # set current coordinate system
        e = self.projector.set_coordinate_system(self.coordinate_system) # set current coordinate system
        rospy.loginfo(e)
        rospy.loginfo("CURRENT coordinate system: [{}] ".format(self.coordinate_system))
        self.projector.get_coordinate_systems() # print CURRENT cs list
        self.projector.show_coordinate_system(self.coordinate_system,req.secs.data)
        
        e = self.projector.unhide_shape(self.coordinate_system + "_origin","polyline","axis_x")
        rospy.loginfo(e)
        e = self.projector.unhide_shape(self.coordinate_system + "_origin","polyline","axis_y")
        rospy.loginfo(e)

        e = self.projector.start_projection(self.coordinate_system)
        rospy.loginfo(e)
        input("PROJECTING COORDINATE SYSTEM ORIGIN AXES. PRESS ENTER TO FINISH.")
        e = self.projector.stop_projection()
        rospy.loginfo(e)

        e = self.projector.hide_shape(self.coordinate_system + "_origin","polyline","axis_x")
        rospy.loginfo(e)
        e = self.projector.hide_shape(self.coordinate_system + "_origin","polyline","axis_y")
        rospy.loginfo(e)
        return ShowCsResponse(Bool(True))

    def remove_coord_sys_cb(self,req):
        rospy.loginfo("Received request to remove [{}] coordinate system".format(req.cs_name.data))
        e = self.projector.remove_coordinate_system(req.cs_name.data) 
        rospy.loginfo(e)
        self.coordinate_system = ""
        rospy.loginfo("Current coordinate system [{}]. Set other coordinate system or define a new one before continue".format(self.coordinate_system))
        return RemovCsResponse(Bool(True))

    def add_shape_cb(self,req):
        rospy.loginfo("Received request to add a: '{}' at the current coordinate system: [{}] ".format(req.shape_type.data, self.coordinate_system))
        # self.projector.get_coordinate_systems()
        
        if req.shape_type.data == "polyline":
            rospy.loginfo("Creating polyline shape")
            e = self.projector.create_polyline(self.coordinate_system,req.projection_group_name.data,req.shape_id.data,req.x.data,req.y.data,req.angle.data,req.r.data)
            rospy.loginfo(e)
            print("Projecting polyline for 5 seconds in order to check the shape")
            self.projector.start_projection(self.coordinate_system)
            time.sleep(5)
            self.projector.stop_projection()
        # elif req.shape_type.data == "circle":
        #     rospy.loginfo("Creating circle shape")
        #     e = self.projector.create_circle(req.projection_group_name.data,req.shape_id.data,req.x.data,req.y.data,req.r.data)
        #     rospy.loginfo(e)
        else: 
            return ShapeParamsResponse(Bool(False))
        return ShapeParamsResponse(Bool(True))
    
    def hide_shape_cb(self,req):
        shape_name = req.projection_group_name.data + "/my_" + req.shape_type.data + "_" + req.shape_id.data
        rospy.loginfo("Received request to deactivate the '{}' from the current coordinate system: [{}] ".format(shape_name, self.coordinate_system))
        e = self.projector.hide_shape(req.projection_group_name.data,req.shape_type.data,req.shape_id.data)
        rospy.loginfo(e)
        return HideShapeResponse(Bool(True))
        
    def remove_shape_cb(self,req):
        shape_name = req.projection_group_name.data + "/my_" + req.shape_type.data + "_" + req.shape_id.data
        rospy.loginfo("Received request to remove the '{}' from the [{}] coordinate system".format(shape_name, self.coordinate_system))
        e = self.projector.remove_shape(req.projection_group_name.data,req.shape_type.data,req.shape_id.data)
        rospy.loginfo(e)
        return RemovShapeResponse(Bool(True))

    def projection_start_cb(self,req):
        e = self.projector.start_projection(self.coordinate_system)
        rospy.loginfo(e)
        return TriggerResponse(True,"Started")
    
    def projection_stop_cb(self,req):
        e = self.projector.stop_projection()
        rospy.loginfo(e)
        return TriggerResponse(True,"Stopped")

def main():
    ProjectionNode()

if __name__ == '__main__':
    main()