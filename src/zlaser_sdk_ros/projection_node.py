#!/usr/bin/env python3

import rospy
import rospkg
import os
import time
from std_srvs.srv import Trigger, TriggerResponse
from projector_manager import ProjectorManager
#from zlaser_sdk_ros.projector_manager import ProjectorManager

class ProjectionNode:
    def __init__(self):
        self.node_name = 'projection_node'
        rospy.init_node(self.node_name)
        rospy.loginfo("main")
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('zlaser_sdk_ros')
        self.setup_path = self.pkg_path + "/scripts/set_up_projector.py"
        self.lic_path = self.pkg_path + "/lic/1900027652.lic"

        # Create projector object
        self.proyector = ProjectorManager()

        # Open connection service
        self.cnt_srv = rospy.Service('/projector_srv/connect', Trigger, self.connectionCb)
        self.discnt_srv = rospy.Service('/projector_srv/disconnect', Trigger, self.disconnectionCb)
        self.lic_srv = rospy.Service('/projector_srv/load_license', Trigger, self.transferLicenseCb)
        self.setup_srv = rospy.Service('/projector_srv/setup', Trigger, self.setupCb)
        self.cs_srv = rospy.Service('/projector_srv/cs', Trigger, self.defineCoordSysCb)
        
        # License to projector
        

        # Define coordinate system
        # os.system("python3 " + self.setup_path + " &")

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
            rospy.logwarn("License is not valid. Load a new one")      
            return TriggerResponse(False,"end setup")    
        else:
            rospy.loginfo("License is valid...")
            # check coordinate system
            cs = self.proyector.getCoordinateSystems()
            rospy.loginfo("Available coordinate systems: {}".format(cs))
        return TriggerResponse(True,"end setup")

    def transferLicenseCb(self,req):        
        self.proyector.license_path = self.lic_path
        e = self.proyector.transferLicense()
        rospy.loginfo(e)
        return TriggerResponse(True,"license loaded")

    def defineCoordSysCb(self,req):
        self.proyector.do_register_coordinate_system = True
        self.proyector.defineCoordinateSystem()
        cs = self.proyector.getCoordinateSystems()
        rospy.loginfo("Available coordinate systems: {}".format(cs))
        rospy.loginfo("Projecting coordinate system")
        self.proyector.showCoordinateSystem(5)

def main():
    ProjectionNode()

if __name__ == '__main__':
    main()