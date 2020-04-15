#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Helper module for python thrift interface to ZLP Service.
    This module contains utility classes and methods which ease the usage of the thrift interface to ZLP Service."""

import os
import time
import math
import socket
import copy
import logging
from threading import Thread

import thriftpy
from thriftpy.protocol import TBinaryProtocolFactory
from thriftpy.server import TThreadedServer, TSimpleServer
from thriftpy.thrift import TProcessor, TClient
from thriftpy.transport import TBufferedTransportFactory, TServerSocket, TSocket

logging.basicConfig(format="%(asctime)s %(name)s %(levelname)s: %(message)s")
log = logging.getLogger(__name__)
log.setLevel(logging.INFO)

_interface_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "interface.thrift")
thrift_interface = thriftpy.load(_interface_file, module_name="zlaser_thrift")

class EventChannelInterfaceHandler(object):
    """This class implement the functions of ClientEventChannel thrift interface."""

    def __init__(self):
        """Initialize the EventChannelInterfaceHandler object."""
        self.property_changed_callback = lambda x, y: None
        self.geo_tree_changed_callback = lambda x, y: None
        self.service_state_changed_callback = lambda x, y: None
        self.function_module_changed_callback = lambda x, y, z: None
        self.rc_command_received_callback = lambda a, b, c, d, e: None
        self.onReflectionStateChanged_callback = lambda a, b: None

    def PropertyChanged(self, name, value):
        self.property_changed_callback(name, value)

    def GeoTreeChanged(self, changed_flags, element_names):
        self.geo_tree_changed_callback(changed_flags, element_names)

    def ServiceStateChanged(self, oldState, newState):
        self.service_state_changed_callback(oldState, newState)

    def FunctionModuleStateChanged(self, functionModID, oldState, newState):
        self.function_module_changed_callback(functionModID, oldState, newState)

    def RemoteControlFrameReceived(self, rc_id, command, toggle, projector, timestamp):
        self.rc_command_received_callback(rc_id, command, toggle, projector, timestamp)

    def onReflectionStateChanged(self, elementName, state):
        self.onReflectionStateChanged_callback(elementName, state)

class ThriftClient(TClient):
    """This class implement the functions to carry out the connection with the ZLP Service."""
    
    def __init__(self, event_handler=EventChannelInterfaceHandler()):
        """Initialize the ThriftClient object.

            Args:
                object event_handler: ClientEventChannel thrift interface"""
        self._event_channel = None
        self._event_channel_handler = event_handler

    def init_client(self, ip, port):
        """Establish a connection to thrift server of ZLP Service.
            
            Args:
                string ip: ipv6 network address of ZLP-Service
                string port: port number on which ZLP-Service listens for requests"""
        client_socket = TSocket(ip, port, socket_family=socket.AF_INET, socket_timeout=50000)
        transport = TBufferedTransportFactory().get_transport(client_socket)
        protocol = TBinaryProtocolFactory().get_protocol(transport)
        transport.open()
        super().__init__(thrift_interface.ServiceInterface, protocol)

    def init_event_channel(self):
        """Create a thrift server and register it at ZLP Service to receive events."""
        if self._event_channel_handler and not self._event_channel:
            processor = TProcessor(thrift_interface.ClientEventChannel, self._event_channel_handler)
            server_socket = TServerSocket(host="0.0.0.0", port=0, socket_family=socket.AF_INET, client_timeout=200000)
            server_socket.client_timeout = 1000*60*10 
            self._event_channel = TSimpleServer(processor, server_socket)

            t = Thread(target=self._event_channel.serve, daemon=True)
            t.start()

            time.sleep(1)
            connection = self._event_channel.trans.sock.getsockname()
            self.ConnectClientEventChannel(connection[1])

    def set_property_changed_callback(self, callback):
        if self._event_channel_handler:
            self._event_channel_handler.property_changed_callback = callback
        else:
            raise ValueError("Error: Can't install callback, because event_handler = none!")

    def set_geotree_changed_callback(self, callback):
        if self._event_channel_handler:
            self._event_channel_handler.geo_tree_changed_callback = callback
        else:
            raise ValueError("Error: Can't install callback, because event_handler = none!")

    def set_function_module_state_changed_callback(self, callback):
        if self._event_channel_handler:
            self._event_channel_handler.function_module_changed_callback = callback
        else:
            raise ValueError("Error: Can't install callback, because event_handler = none!")

    def set_rc_command_received_callback(self, callback):
        if self._event_channel_handler:
            self._event_channel_handler.rc_command_received_callback = callback
        else:
            raise ValueError("Error: Can't install callback, because event_handler = none!")

    def set_reflection_state_changed_callback(self, callback):
        if self._event_channel_handler:
            self._event_channel_handler.onReflectionStateChanged_callback = callback
        else:
            raise ValueError("Error: Can't install callback, because event_handler = none!")

class ProjectorClient(object):
    """This class implement the functions to perform elementary operations with the projector."""

    def __init__(self):
        """Initialize the ProjectorClient object."""
        self.projector_id = ""
        self.module_id = ""

        self.__thrift_client = ThriftClient()

    def get_thrift_client(self):
        """Return the object generated to communicate with the projector.
            
            Returns:
                object ThriftClient(): object with the generated client to communicate with the projector"""
        try:
            return self.__thrift_client
        
        except Exception as e:
            return e

    def connect(self,server_IP,connection_port):
        """Create and connect the client to thrift server (located at projector) of ZLP-Service and establish an event channel if needed.

            Args:
                string server_IP: ipv6 network address of ZLP-Service
                string connection_port: port number on which ZLP-Service listens for requests

            Returns:
                bool success: success value
                string message: information message"""
        try:
            self.__thrift_client.init_client(server_IP, connection_port)
            self.__thrift_client.init_event_channel()
            success = True 
            message = "Client connected"

        except Exception as e:
            success = False 
            message = e
    
        return success,message

    def disconnect(self):
        """Disconnect from ZLP Service thrift server and close own event server.
        
            Returns:
                bool success: success value
                string message: information message"""
        try: 
            self.__thrift_client.RemoveGeoTreeElem("") 
            self.__thrift_client.FunctionModuleRelease(self.module_id)
            
            self.__thrift_client.DisconnectClientEventChannel()
            self.__thrift_client.close()

            if self.__thrift_client._event_channel:
                self.__thrift_client._event_channel.close()
        
            success = True 
            message = "Projector disconnected"

        except Exception as e:
            success = False 
            message = e

        return success,message

    def scan_projectors(self, scan_addresses=""):
        """Scan the network for projectors. Get a list of active projectors.

            Args:
                string scan_addresses: addresses or address to scan

            Returns:
                list serial_list: serial numbers of the found projectors
                bool success: success value
                string message: information message"""
        try:
            self.__thrift_client.SetProperty("config.projectorManager.cmdGetProjectors.scan", "1")
            self.__thrift_client.SetProperty("config.projectorManager.cmdGetProjectors.scanAddresses", scan_addresses)
            self.__thrift_client.SetProperty("config.projectorManager.cmdGetProjectors", "1")
            serial_list = self.__thrift_client.GetProperty("config.projectorManager.cmdGetProjectors.result.entries")
            self.__thrift_client.SetProperty("config.projectorManager.cmdGetProjectors.scan", "0")
            
            if serial_list:
                serial_list = serial_list.split(" ")
                success = True 
                message = ""
            else:
                serial_list = []
                success = False
                message = "No projectors found"

        except Exception as e:
            serial_list = []
            success = False
            message = e

        return serial_list,success,message

    def activate_projector(self,projector_IP):
        """Set properties to activate a projector.
        
            Args:
                string projector_IP: address of the projector to scan

            Returns:
                string projector_id: serial number of the found projector
                bool success: success value
                string message: information message"""
        try:
            projectors,s,m = self.scan_projectors(projector_IP)
            if s:
                self.projector_id = projectors[0]
                self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector.serial", self.projector_id)
                self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector.active", "1")
                self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector", "1")
                success = True 
                message = "Projector activated"
            else:
                success = False 
                message = m
        
        except Exception as e:
            success = False 
            message = e
        
        return self.projector_id,success,message

    def deactivate_projector(self):
        """Set properties to deactivate a projector.
        
            Returns:
                bool success: success value
                string message: information message"""
        try:
            projector_property_path = "config.projectorManager.projectors." + self.projector_id
            self.__thrift_client.SetProperty(projector_property_path + ".cmdShowProjection.show", "0")
            self.__thrift_client.SetProperty(projector_property_path + ".cmdShowProjection", "1")

            self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector.serial", self.projector_id)
            self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector.active", "0")
            self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector", "1")

            success = True 
            message = "Projector deactivated:" + self.projector_id

        except Exception as e:
            success = False 
            message = e

        return success,message

    def transfer_license(self, lic_path):
        """Transfer license file to projector.
        
            Args:
                string lic_path: license file path

            Returns:
                bool success: success value
                string message: information message"""
        try:
            license_path = os.path.abspath(lic_path)
            license_file = os.path.basename(license_path)
            s,m = self.transfer_file(license_path, license_file, True)
            if s:
                self.__thrift_client.LoadLicense(license_file)
                success = True 
                message = "License transfered"
            else:
                success = False 
                message = m
        
        except thrift_interface.CantWriteFile as e:
            success = False
            message = e
        
        except FileNotFoundError as e:
            success = False
            message = e
        
        return success,message

    def transfer_file(self, local_path, remote_file, overwrite=False): # private?
        """Transfer data of the local license file to remote file at ZLP-Service.
        
            Args:
                string local_path: normalized absolutized version of the pathname path 
                string remote_file: base name of a normalized absolutized version of the pathname path
                bool overwrite: overwrite data over remote file parameter

            Returns:
                bool success: success value
                string message: information message"""
        try:
            content = open(local_path, 'r').read()
            self.__thrift_client.TransferDataToFile(content, remote_file, overwrite)
            success = True
            message = "File transfered"
        
        except Exception as e:
            success = False
            message = e

        return success,message

    def check_license(self):
        """Check if license is valid.
        
            Returns:
                bool thrift_client.CheckLicense(): return True for success, False otherwise """
        try:
            success = self.__thrift_client.CheckLicense()
            if success:
                message = "License is valid"
            else:
                message = "License is not valid"
        except Exception as e:
            success = False
            message = e

        return success,message

    def function_module_create(self):
        """Create function module to operate with GeoTreeElements (coordinate systems and shapes).
        
            Returns:
                string module_id: function module identification name
                bool success: success value
                string message: information message"""
        try:
            self.module_id = self.__thrift_client.FunctionModuleCreate("zFunctModRegister3d", "3DReg")
            success = True
            message = "Function module created"
        
        # except thrift_interface.FunctionModuleClassNotRegistered as e:
        #     return ("FunctionModuleClassNotRegistered: " + e.which)
        # except thrift_interface.FunctionModulePropertyBranchAlreadyInUse as e:
        #     return ("FunctionModulePropertyBranchAlreadyInUse: " + e.branchName)
        # except thrift_interface.FunctionModuleClassNotLicensed as e:
        #     return ("FunctionModuleClassNotLicensed: " + e.which)

        except Exception as e:
            success = False
            message = e
        
        return self.module_id,success,message

    def start_project(self, coord_sys):
        """Start projection on the surface of all figures (shapes) that belong to the active coordinate system.
            
            Args:
                string coord_sys: name of the current coordinate system

            Returns:
                bool success: success value
                string message: information message"""
        try:
            ref_obj_name = "RefObj_" + coord_sys
            ref_obj = self.__thrift_client.GetGeoTreeElement(ref_obj_name)
            if ref_obj.activated == False or len(self.__thrift_client.GetGeoTreeIds()) <= len(self.__thrift_client.GetCoordinatesystemList()):
                success = False
                message = "Coordinate_system is not activated or nothing to project"
            else:
                self.__thrift_client.TriggerProjection()
                success = True
                message = "Projecting"
        
        except Exception as e:
            success = False
            message = e
        
        return success,message

    def stop_project(self): 
        """Stop projection of all figures.
            
            Returns:
                bool success: success value
                string message: information message"""
        try:
            projector_property_path = "config.projectorManager.projectors." + self.projector_id
            self.__thrift_client.SetProperty(projector_property_path + ".cmdShowProjection.show", "0")
            self.__thrift_client.SetProperty(projector_property_path + ".cmdShowProjection", "1")
            success = True
            message = "Projection stopped"
        
        except Exception as e:
            success = False
            message = e
        
        return success,message

class GeometryTool(object):
    """This class implement functions to generate basic mathematical tools."""

    def create_matrix4x4(self):
        """Initialize 4x4 matrix.
            
            Returns:
                struct mat: matrix struct initialized with empty values"""
        mat = thrift_interface.Matrix4x4(list())
        return mat

    def create_2d_point(self, x=0, y=0):
        """Initialize 2-dimension array.
            
            Args:
                double x: x position value
                double y: y position value

            Returns:
                bool thrift_interface.Vector2D: struct with the values of the 2 axis (x,y)"""
        return thrift_interface.Vector2D(x, y)

    def create_3d_point(self, x=0, y=0, z=0):
        """Initialize 3-dimension array.
            
            Args:
                double x: x position value
                double y: y position value
                double z: z position value

            Returns:
                struct thrift_interface.Vector3D: struct with the values of the 3 axis (x,y,z)"""
        return thrift_interface.Vector3D(x, y, z)

class CoordinateSystemParameters(object):
    """This class is used as data structure with the necessary information to define a coordinate system."""
    
    def __init__(self,cs):
        """Initialize the CoordinateSystemParameters object.
        
            Args:
                struct cs: struct with the necessary parameters to create the coordinate system"""
        self.name    = cs.name_cs.data
        self.d       = cs.distance.data
        self.x1      = cs.x1.data
        self.y1      = cs.y1.data
        self.x2      = cs.x2.data
        self.y2      = cs.y2.data
        self.x3      = cs.x3.data
        self.y3      = cs.y3.data
        self.x4      = cs.x4.data
        self.y4      = cs.y4.data
        self.T1_x    = cs.T1_x.data
        self.T1_y    = cs.T1_y.data

class ProjectionElementParameters(object):
    """This class is used as data structure with the necessary information to create a projection element."""
    
    # def __init__(self,proj_elem):
    #     """Initialize the ProjectionElementParameters object.
        
    #         Args:
    #             struct proj_elem: struct with the necessary parameters to create the projection element"""
    #     self.shape_type            = proj_elem.shape_type.data
    #     self.projection_group_name = proj_elem.projection_group_name.data
    #     self.shape_id              = proj_elem.shape_id.data
    #     self.x                     = proj_elem.x.data
    #     self.y                     = proj_elem.y.data
    #     self.angle                 = proj_elem.angle.data
    #     self.length                = proj_elem.length.data

    def __init__(self,proj_elem):
        """Initialize the ProjectionElementParameters object.
        
            Args:
                struct proj_elem: struct with the necessary parameters to create the projection element"""
        self.proj_elem             = proj_elem
        self.shape_type            = ""
        self.projection_group_name = ""
        self.shape_id              = ""
        self.x                     = 0.0
        self.y                     = 0.0
        self.angle                 = 0.0
        self.length                = 0.0

    def set_params(self):
        """Set the ProjectionElementParameters values."""
        self.shape_type            = self.proj_elem.shape_type.data
        self.projection_group_name = self.proj_elem.projection_group_name.data
        self.shape_id              = self.proj_elem.shape_id.data
        self.x                     = self.proj_elem.x.data
        self.y                     = self.proj_elem.y.data
        self.angle                 = self.proj_elem.angle.data
        self.length                = self.proj_elem.length.data

class CoordinateSystem(object):
    """This class implement the functions related with coordinate systems management."""
    
    def __init__(self, projector_id, module_id, thrift_client):
        """Initialize the CoordinateSystem object.
        
            Args:
                string projector_id: serial number of the projector
                string module_id: function module identification name
                object thrift_client: object with the generated client to communicate with the projector"""
        self.__thrift_client = thrift_client 
        self.__geometry_tool = GeometryTool()

        self.projector_id = projector_id
        self.module_id = module_id
        self.reference_object_list = []

    def coordinate_system_list(self):
        """Get list of current available coordinate systems from projector.
        
            Returns:
                list cs_list: names list of available coordinate systems
                bool success: success value
                string message: information message"""
        try:
            cs_list = self.__thrift_client.GetCoordinatesystemList()
            success = True
            message = ""
        
        except Exception as e:
            cs_list = []
            success = False
            message = e
        
        return cs_list,success,message

    def create_reference_object(self):
        """Generate new reference object.
            
            Returns:
                struct ref_obj: reference object struct fields initialized"""
        ref_obj = thrift_interface.Referenceobject()
        ref_obj.name = ""
        ref_obj.activated = False
        ref_obj.fieldTransMat = self.__geometry_tool.create_matrix4x4()
        ref_obj.refPointList = []
        ref_obj.projectorID = ""
        ref_obj.coordinateSystem = ""
        return ref_obj

    def create_reference_point(self, name, x, y, z=0, active=True):
        """Generate new reference point.

            Args:
                string name: reference point name
                double x: x position value
                double y: y position value
                double z: z position value
                bool active: activate reference point parameter
            
            Returns:
                struct ref_point: reference point struct fields initialized"""
        ref_point = thrift_interface.Referencepoint()
        ref_point.name = name
        ref_point.refPoint = self.__geometry_tool.create_3d_point(x, y, z)
        ref_point.activated = active
        ref_point.tracePoint = self.__geometry_tool.create_2d_point()
        ref_point.crossSize = self.__geometry_tool.create_2d_point()
        ref_point.distance = 0
        return ref_point

    def define_cs(self,cs):
        """Generate new coordinate system.

            Args:
                struct cs: struct with the necessary parameters to create the coordinate system 
            
            Returns:
                string cs.name: name of the coordinate system generated
                bool success: success value
                string message: information message"""
        try:
            reference_object = self.create_reference_object()
            reference_object.name = "RefObj_" + cs.name
            reference_object.coordinateSystem = cs.name
            reference_object.projectorID = self.projector_id
            
            T1_x = cs.T1_x
            T1_y = cs.T1_y
            T2_x = T1_x + abs((cs.x2 - cs.x1))
            T2_y = T1_y
            T3_x = T1_x + abs((cs.x3 - cs.x1))
            T3_y = T1_y + abs((cs.y3 - cs.y1))
            T4_x = T1_x
            T4_y = T1_y + abs((cs.y4 - cs.y1))

            # scale_factor = ??? T_x,y * 2 # manteniendo la proporción

            reference_object.refPointList = [   self.create_reference_point("T1", T1_x, T1_y),
                                                self.create_reference_point("T2", T2_x, T2_y),
                                                self.create_reference_point("T3", T3_x, T3_y),
                                                self.create_reference_point("T4", T4_x, T4_y)]
            
            d = cs.d
            cross_size_x = d * 0.02
            cross_size_y = d * 0.02

            cross_size = self.__geometry_tool.create_2d_point(cross_size_x,cross_size_y)
            
            reference_object = self.__define_reference_point(reference_object,cross_size,0,d,cs.x1,cs.y1)
            reference_object = self.__define_reference_point(reference_object,cross_size,1,d,cs.x2,cs.y2)
            reference_object = self.__define_reference_point(reference_object,cross_size,2,d,cs.x3,cs.y3)
            reference_object = self.__define_reference_point(reference_object,cross_size,3,d,cs.x4,cs.y4)
            
            self.__add_ref_object(reference_object)

            success = True
            message = "Cordinate system defined correctly"

        except Exception as e:
            success = False 
            message = e

        return cs.name,success,message

    def __define_reference_point(self,reference_object,cross_size,n,d,x,y):
        """Fill other fields of the coordinate system structure.

            Args:
                struct reference_object: current coordinate system struct
                struct cross_size: struct with the dimensions of the crosses
                int n: vector index
                float d: distance between the projection surface and the projector
                float x: value of the x-axis coordinate
                float y: value of the y-axis coordinate
            
            Returns:
                struct reference_object: coordinate system parameters structure updated """
        reference_object.refPointList[n].tracePoint.x = x
        reference_object.refPointList[n].tracePoint.y = y
        reference_object.refPointList[n].distance = d
        reference_object.refPointList[n].activated = True
        reference_object.refPointList[n].crossSize = cross_size
        return reference_object

    def __add_ref_object(self,ref_obj):
        """Add new coordinate system to the list.

            Args:
                struct ref_obj: current reference object structure of the generated coordinate system"""
        self.reference_object_list.append(ref_obj)

    def set_cs(self,coord_sys): 
        """Activate the new generated coordinate system and deactivate the other existing coordinate systems at the projector.

            Args:
                string coord_sys: name of the new coordinate system
            
            Returns:
                bool success: success value
                string message: information message"""
        try:
            ref_obj_name = "RefObj_" + coord_sys 

            for i in range (0,len(self.reference_object_list)):
                if self.reference_object_list[i].name == ref_obj_name:
                    index = i
                else:
                    # print("{} DEACTIVATED" .format(self.reference_object_list[i].name))

                    self.__ref_obj_state(False,self.reference_object_list[i])

                    # self.reference_object_list[i].activated = False
                    # self.__thrift_client.SetReferenceobject(self.reference_object_list[i])
                    # self.__thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", self.reference_object_list[i].name)
                    # self.__thrift_client.FunctionModuleSetProperty(self.module_id, "runMode", "1")
                    # self.__thrift_client.FunctionModuleRun(self.module_id)
            
            # print("{} ACTIVATED" .format(self.reference_object_list[index].name))

            self.__ref_obj_state(True,self.reference_object_list[index])

            # self.reference_object_list[index].activated = True
            # self.__thrift_client.SetReferenceobject(self.reference_object_list[index])
            # self.__thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", self.reference_object_list[index].name)
            # self.__thrift_client.FunctionModuleSetProperty(self.module_id, "runMode", "1")
            # self.__thrift_client.FunctionModuleRun(self.module_id)

            success = True
            message = coord_sys + " set as current coordinate system"

        except Exception as e:
            success = False 
            message = e

        return success,message

    def __ref_obj_state(self, state, ref_obj):
        """Activate or deactivate a reference object.

            Args:
                bool state: activation/deactivation parameter
                struct reference_object: reference object to be activated/deactivated"""
        ref_obj.activated = state
        self.__thrift_client.SetReferenceobject(ref_obj)
        self.__thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", ref_obj.name)
        self.__thrift_client.FunctionModuleSetProperty(self.module_id, "runMode", "1")
        self.__thrift_client.FunctionModuleRun(self.module_id)

    def register_cs(self,coord_sys):
        """Register the new coordinate system at the projector once it has been generated and activated.

            Args:
                string coord_sys: name of the coordinate system
            
            Returns:
                bool success: success value
                string message: information message"""
        try:
            self.__thrift_client.FunctionModuleSetProperty(self.module_id, "runMode", "1")
            self.__thrift_client.FunctionModuleRun(self.module_id)
            state = self.__thrift_client.FunctionModuleGetProperty(self.module_id, "state")

            if state != "1":
                success = False
                message = "Function module is not in idle state, hence an error has occured."
            else:
                success = True
                message = "Coordinate system [" + coord_sys + "] registered on projector."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def show_cs(self,coord_sys,secs):
        """Project a coordinate system on the projection surface.

            Args:
                string coord_sys: name of the coordinate system
                int secs: number of seconds the projection lasts

            Returns:
                bool success: success value
                string message: information message"""
        try:
            self.__thrift_client.FunctionModuleSetProperty(self.module_id,"showAllRefPts","1")
            time.sleep(secs)
            self.__thrift_client.FunctionModuleSetProperty(self.module_id,"showAllRefPts","0")
            
            success = True
            message = "Finished to show coordinate system"
        
        except Exception as e:
            success = False 
            message = e

        return success,message

    def remove_cs(self,coord_sys):
        """Delete a coordinate system.
            
            Args:
                string coord_sys: name of the coordinate system
                
            Returns:
                bool success: success value
                string message: information message"""
        try:
            reference_object_name = "RefObj_" + coord_sys
            self.__thrift_client.RemoveGeoTreeElem(reference_object_name)
            success = True
            message = "Coordinate system [" + coord_sys + "] removed."

        except Exception as e:
            success = False 
            message = e

        return success,message

class ProjectionElementControl(object):
    """This class implement the functions related with figures projection."""
    
    def __init__(self,module_id, thrift_client):
        """Initialize the ProjectionElementControl object.
        
            Args:
                string module_id: function module identification name
                object thrift_client: object with the generated client to communicate with the projector"""
        self.__thrift_client = thrift_client
        self.__geometry_tool = GeometryTool()
        self.module_id = module_id

        self.default_projection_element = thrift_interface.ProjectionElement()
        self.default_projection_element.pen = 0
        self.default_projection_element.coordinateSystemList = []
        self.default_projection_element.projectorIDList = []
        self.default_projection_element.userTrans = self.__geometry_tool.create_matrix4x4()
        self.default_projection_element.activated = True

    def init_projection_element(self, elem):
        """Initialize new projection element.

            Args:
                struct elem: projection element struct to initialize
            
            Returns:
                struct elem: projection element struct with fields initialized"""
        elem.coordinateSystemList = copy.deepcopy(self.default_projection_element.coordinateSystemList)
        elem.projectorIDList = copy.deepcopy(self.default_projection_element.projectorIDList)
        elem.userTrans = copy.deepcopy(self.default_projection_element.userTrans)
        elem.activated = self.default_projection_element.activated
        elem.pen = self.default_projection_element.pen
        return elem

    # def create_projection_element(self, name):
    #     """Generate new reference object.
            
    #         Args:
    #             string name:  projection element name

    #         Returns:
    #             struct projection_element: reference object struct fields initialized"""
    #     projection_element = copy.deepcopy(self.default_projection_element)
    #     projection_element.name = name
    #     return projection_element

    def create_polyline(self, name):
        """Generate new polyline.
            
            Args:
                string name:  polyline name
            
            Returns:
                struct polyline: polyline struct with fields initialized"""
        polyline = thrift_interface.Polyline()
        polyline = self.init_projection_element(polyline)
        polyline.name = name
        polyline.polylineList = []
        return polyline

    def define_polyline(self,coord_sys,proj_elem_params):
        """Create a new line to project.

            Args:
                string coord_sys: name of coordinate system which the new projection element will be added
                string projection_group: name of the projection group
                string id: name of the shape identificator
                float x: position at x-axis of the line initial point
                float y: position at y-axis of the line initial point
                float angle: line angle value
                float length: length of the line
                
            Returns:
                bool success: success value
                string message: information message"""
        try:
            projection_group = proj_elem_params.projection_group_name
            id               = proj_elem_params.shape_id
            x                = proj_elem_params.x
            y                = proj_elem_params.y
            angle            = proj_elem_params.angle
            length           = proj_elem_params.length

            polyline_name = projection_group + "/my_polyline_" + id
            polyline = self.create_polyline(polyline_name)

            linestring = [ self.__geometry_tool.create_3d_point(x, y),
                           self.__geometry_tool.create_3d_point(x+length*math.cos(angle*math.pi/180), y+length*math.sin(angle*math.pi/180))]
            
            polyline.polylineList = [linestring]
            polyline.activated = True
            polyline.coordinateSystemList = [coord_sys]
        
            self.__thrift_client.SetPolyLine(polyline)
            
            success = True
            message = polyline_name + " polyline created."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def deactivate_shape(self,proj_elem_params): 
        """Hide (deactivate) a figure from a group of the active coordinate system.

            Args:
                string projection_group: name of the projection group
                string shape_name: type of figure (polyline, circle, etc.)
                string id: name of the shape identificator
                
            Returns:
                bool success: success value
                string message: information message"""
        try:
            shape_type       = proj_elem_params.shape_type
            projection_group = proj_elem_params.projection_group_name
            id               = proj_elem_params.shape_id

            if shape_type == "polyline":
                name = projection_group + "/my_" + shape_type + "_" + id
                polyline = self.__thrift_client.GetPolyLine(name)
                polyline.activated = False
                self.__thrift_client.SetPolyLine(polyline)
                
                success = True
                message = "Polyline" + name + "deactivated."

            else:
                success = False
                message = "Shape name does not exist."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def reactivate_shape(self,proj_elem_params): 
        """Unhide (activate) a figure from a group of the active coordinate system.

            Args:
                string projection_group: name of the projection group
                string shape_name: type of figure (polyline, circle, etc.)
                string id: name of the shape identificator
                
            Returns:
                bool success: success value
                string message: information message"""
        try:
            shape_type       = proj_elem_params.shape_type
            projection_group = proj_elem_params.projection_group_name
            id               = proj_elem_params.shape_id

            if shape_type == "polyline":
                name = projection_group + "/my_" + shape_type + "_" + id
                polyline = self.__thrift_client.GetPolyLine(name)
                polyline.activated = True
                self.__thrift_client.SetPolyLine(polyline)
                
                success = True
                message = "Polyline" + name + "reactivated."

            else:
                success = False
                message = "Shape name does not exist."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def delete_shape(self,proj_elem_params):
        """Delete a figure from the active coordinate system.

            Args:
                string projection_group: name of the projection group
                string shape_name: type of figure (polyline, circle, etc.)
                string id: name of the shape identificator
                
            Returns:
                bool success: success value
                string message: information message"""
        try:
            shape_type       = proj_elem_params.shape_type
            projection_group = proj_elem_params.projection_group_name
            id               = proj_elem_params.shape_id

            self.__thrift_client.RemoveGeoTreeElem(projection_group + "/my_" + shape_type + "_" + id)
            success = True
            message = "Shape removed"

        except Exception as e:
            success = False 
            message = e

        return success,message

    # def cs_origin_axes(self,coord_sys,cs):
    #     """Generate coordinate system origin x and y axes.
            
    #         Args:
    #             string coord_sys: name of the coordinate system
    #             struct cs: struct with the necessary parameters to create the coordinate system
                
    #         Returns:
    #             bool success: success value
    #             string message: information message"""
    #     try:
            
    #         self.define_polyline(coord_sys, coord_sys + "_origin","axis_x",cs.T1_x,cs.T1_y,0,50) 
    #         self.define_polyline(coord_sys, coord_sys + "_origin","axis_y",cs.T1_x,cs.T1_y,90,50)
    #         success = True
    #         message = ""

    #     except Exception as e:
    #         success = False 
    #         message = e

    #     return success,message
