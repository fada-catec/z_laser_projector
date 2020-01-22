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
        self.setup_path = self.pkg_path + "/scripts/set_up_projector.py" #not used
        self.lic_path = self.pkg_path + "/lic/1900027652.lic"

        # Create projector object
        self.proyector = ProjectorManager()

        # Open services
        self.cnt_srv     = rospy.Service('/projector_srv/connect', Trigger, self.connectionCb)
        self.discnt_srv  = rospy.Service('/projector_srv/disconnect', Trigger, self.disconnectionCb)
        self.lic_srv     = rospy.Service('/projector_srv/load_license', Trigger, self.transferLicenseCb)
        self.setup_srv   = rospy.Service('/projector_srv/setup', Trigger, self.setupCb)
        self.cs_srv      = rospy.Service('/projector_srv/cs', Trigger, self.defineCoordSysCb)
        self.project_srv = rospy.Service('/projector_srv/project', ProjectionShape, self.projectionCb)
        self.stop_srv    = rospy.Service('/projector_srv/stop', Trigger, self.projectionStopCb)
        self.show_srv    = rospy.Service('/projector_srv/show', Trigger, self.showCb)

        rospy.spin()

    def connectionCb(self,req):
        rospy.loginfo("Received request to start projector")
        e = self.proyector.clientServerConnect()
        rospy.loginfo(e)
        e = self.proyector.activate()
        rospy.loginfo(e)
        return TriggerResponse(True,str(e))
    
    def disconnectionCb(self,req):
        rospy.loginfo("Received request to stop projector")
        e = self.proyector.deactivate()
        rospy.loginfo(e)
        return TriggerResponse(True,str(e))

    def showCb(self,cs):
        self.projector.showCoordinateSystem(10)

    def setNewCoordSystem(self,cs):
        if len(cs)>1:
            rospy.loginfo("Setting {} as default coordinate system".format(cs[-1]))
            self.proyector.setCoordinateSystem(cs[-1])

    def setupCb(self,req):
        rospy.loginfo("Received request to setup projector")
        # connect to service
        e = self.proyector.clientServerConnect()
        rospy.loginfo(e)
        # activate projector
        e = self.proyector.activate()
        rospy.loginfo(e)
        # check license
        if not self.proyector.checkLicense():
            rospy.logwarn("License is not valid. Load a new one: \n\n rosservice call /projector_srv/load_license")      
            return TriggerResponse(False,"end setup")    
        else:
            rospy.loginfo("License is valid...")
            # check coordinate system
            cs = self.proyector.getCoordinateSystems()
            rospy.loginfo("Available coordinate systems: {}".format(cs))
            # save coordinate system as default
            self.setNewCoordSystem(cs)
        return TriggerResponse(True,"end setup")

    def transferLicenseCb(self,req):        
        self.proyector.license_path = self.lic_path
        e = self.proyector.transferLicense()
        rospy.loginfo(e)
        return TriggerResponse(True,"license loaded")

    def defineCoordSysCb(self,req):
        rospy.loginfo("Received request to create new coordinate system. Please wait for the system to indicate the end")
        # define and register coordinate system
        self.proyector.do_register_coordinate_system = True
        e = self.proyector.defineCoordinateSystem("small_pieza")
        rospy.loginfo(e)
        # show created coordinate system
        cs = self.proyector.getCoordinateSystems()
        rospy.loginfo("Available coordinate systems: {}".format(cs))
        rospy.loginfo("Projecting {} coordinate system".format(cs[-1]))
        self.proyector.showCoordinateSystem(5)
        # save new coordinate system as default
        self.setNewCoordSystem(cs)
        return TriggerResponse(True,"Created coordinate system")            
    
    def projectionCb(self,req):
        rospy.loginfo("Received request to project")
        # get values from service call and pass to projector manager
        x = req.x.data 
        y = req.y.data 
        r = req.r.data
        id = str(req.id.data)
        shape = req.shape.data 
        if shape == "circle":
            rospy.loginfo("Creating circle shape")
            e = self.proyector.createCircle(x,y,r,id)
            rospy.loginfo(e)
            # start projection until stop service is called
            rospy.loginfo(" - Projecting - ")
            self.proyector.startProjection()

        elif shape == "cross":
            rospy.loginfo("Creating cross shape")
            e = self.proyector.createCross(x,y,r,id)
            rospy.loginfo(e)
            # start projection until stop service is called
            rospy.loginfo(" - Projecting - ")
            self.proyector.startProjection()  
        else: 
            return ProjectionShapeResponse(Bool(False))
        return ProjectionShapeResponse(Bool(True))

    def projectionStopCb(self,req):
        # stop projection when service is called 
        self.proyector.stopProjection()
        return TriggerResponse(True,"Stopped")

def main():
    ProjectionNode()

if __name__ == '__main__':
    main()