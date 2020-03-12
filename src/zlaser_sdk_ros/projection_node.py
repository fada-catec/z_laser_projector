#!/usr/bin/env python3

import rospy
import rospkg
import os
import time
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool
from projector_manager import ProjectorManager
from zlaser_sdk_ros.srv import ProjectionShape, ProjectionShapeResponse
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

        # Create projector object
        self.projector = ProjectorManager()

        # Open services
        self.cnt_srv     = rospy.Service('/projector_srv/connect', Trigger, self.connection_cb)
        self.discnt_srv  = rospy.Service('/projector_srv/disconnect', Trigger, self.disconnection_cb)
        self.lic_srv     = rospy.Service('/projector_srv/load_license', Trigger, self.transfer_license_cb)
        self.setup_srv   = rospy.Service('/projector_srv/setup', Trigger, self.setup_cb)

        self.cs_srv      = rospy.Service('/projector_srv/cs', Trigger, self.define_coord_sys_cb)
        self.show_srv    = rospy.Service('/projector_srv/show', Trigger, self.show_coord_sys_cb)

        self.project_srv = rospy.Service('/projector_srv/project', ProjectionShape, self.projection_cb)
        self.stop_srv    = rospy.Service('/projector_srv/stop', Trigger, self.projection_stop_cb)
        

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
            rospy.loginfo("Default coordinate system: {}".format(cs_list[-1]))
            e = self.projector.set_coordinate_system(cs_list[-1]) # set default coordinate system
            rospy.loginfo(e)
            # AQUÍ FALTARÍA -> show_coord_system: name, project points, project axis, print SC properties (position, distance, etc.)
        return TriggerResponse(True,"end setup")



    # def set_coord_system(self,cs):
    #     if len(cs)>1:
    #         rospy.loginfo("Received request to set coordinate system. Setting [{}] as coordinate system".format(cs))
    #         self.projector.set_coordinate_system(cs[-1])

    def show_coord_sys_cb(self,cs):
        self.projector.show_coordinate_system(10)

    def define_coord_sys_cb(self,req):
        rospy.loginfo("Received request to create new coordinate system. Please wait for the system to indicate the end")
        # define and register coordinate system
        self.projector.do_register_coordinate_system = True
        e = self.projector.define_coordinate_system("small_pieza")
        rospy.loginfo(e)
        # show created coordinate system
        cs = self.projector.get_coordinate_systems()
        rospy.loginfo("Available coordinate systems: {}".format(cs))
        rospy.loginfo("Projecting {} coordinate system".format(cs[-1]))
        self.projector.show_coordinate_system(5)
        # save new coordinate system as default
        self.set_coord_system(cs)
        return TriggerResponse(True,"Created coordinate system")            



    def projection_cb(self,req):
        rospy.loginfo("Received request to project")
        # get values from service call and pass to projector manager
        x = req.x.data 
        y = req.y.data 
        r = req.r.data
        id = str(req.id.data)
        shape = req.shape.data 
        if shape == "circle":
            rospy.loginfo("Creating circle shape")
            e = self.projector.create_circle(x,y,r,id)
            rospy.loginfo(e)
            # start projection until stop service is called
            rospy.loginfo(" - Projecting - ")
            self.projector.start_projection()

        elif shape == "cross":
            rospy.loginfo("Creating cross shape")
            e = self.projector.create_cross(x,y,r,id)
            rospy.loginfo(e)
            # start projection until stop service is called
            rospy.loginfo(" - Projecting - ")
            self.projector.start_projection()  
        else: 
            return ProjectionShapeResponse(Bool(False))
        return ProjectionShapeResponse(Bool(True))

    def projection_stop_cb(self,req):
        # stop projection when service is called 
        self.projector.stop_projection()
        return TriggerResponse(True,"Stopped")

def main():
    ProjectionNode()

if __name__ == '__main__':
    main()