#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#!/usr/bin/env python3

# Copyright (c) 2019, FADA-CATEC

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Helper module for python thrift interface to ZLP Service. 
This module contains utility classes and methods which ease the usage of the thrift interface to ZLP Service."""

import os
import sys
import time
import math
import numpy as np
import socket
import copy
import logging
import threading 

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
            event_handler (object): ClientEventChannel thrift interface
        """
        self._event_channel = None
        self._event_channel_handler = event_handler

    def init_client(self, ip, port):
        """Establish a connection to thrift server of ZLP Service. Init client opening sockets and init events handler.
            
        Args:
            ip (str): ipv6 network address of ZLP-Service
            port (str): port number on which ZLP-Service listens for requests
        """
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

            t = threading.Thread(target=self._event_channel.serve, daemon=True)
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

        self.cv = threading.Condition()

    def get_thrift_client(self):
        """Return the object generated to communicate with the projector.
            
        Returns:
            object: object with the generated client to communicate with the projector
        """
        try:
            return self.__thrift_client
        
        except Exception as e:
            return e

    def connect(self,server_IP,connection_port):
        """Create and connect the client to thrift server (located at projector) of ZLP-Service and establish an event channel if needed.

        Args:
            server_IP (str): ipv6 network address of ZLP-Service
            connection_port (str): port number on which ZLP-Service listens for requests

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
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
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
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

    def transfer_license(self, lic_path):
        """Transfer license file to projector.
        Transfer data of the local license file to remote file at ZLP-Service.
        
        local_path (str): normalized absolutized version of the pathname path 
        remote_file (str): base name of a normalized absolutized version of the pathname path
        overwrite (bool): overwrite data over remote file parameter
        
        Args:
            lic_path (str): license file path

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        try:
            license_path = os.path.abspath(lic_path)
            license_file = os.path.basename(license_path)
            content = open(license_path, 'r').read()

            self.__thrift_client.TransferDataToFile(content, license_file, True)
        
        except thrift_interface.CantWriteFile as e:
            success = False
            message = e
        
        except FileNotFoundError as e:
            success = False
            message = e

        self.__thrift_client.LoadLicense(license_file)

        success = True
        message = "License transfered."
        
        return success,message

    def check_license(self):
        """Check if license is valid.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
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

    def scan_projectors(self, scan_addresses=""):
        """Scan the network for projectors. Get a list of active projectors.

        Args:
            scan_addresses (str): addresses or address to scan

        Returns:
            tuple[list, bool, str]: the first value in the returned tuple is a list of serial numbers of the projectors found, the second a bool success value and 
            the third value in the tuple is an information message string
        """
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
    
    def property_changed_callback(self, prop, value):
        self.cv.acquire()
        self.cv.notify()
        self.cv.release()

    def activate_projector(self,projector_IP):
        """Set properties to activate a projector.
        
        Args:
            projector_IP (str): address of the projector to scan

        Returns:
            tuple[str, bool, str]: the first value in the returned tuple is the serial number string of the activated projector,
            the second a bool success value and the third value in the tuple is an information message string
        """
        try:
            projectors,s,m = self.scan_projectors(projector_IP)
            if s:
                self.__thrift_client.set_property_changed_callback(self.property_changed_callback)
                self.__thrift_client.RegisterForChangedProperty("config.licenseState.IsValid")

                self.cv.acquire()
                self.projector_id = projectors[0]
                self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector.serial", self.projector_id)
                self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector.active", "1")
                self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector", "1")
                self.cv.wait()
                self.cv.release()
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
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
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

    def function_module_create(self):
        """Create function module to operate with GeoTreeElements (coordinate systems and shapes).

        Returns:
            tuple[str, bool, str]: the first value in the returned tuple is the function module identification name string, 
            the second is a bool success value and the third value in the tuple is an information message string
        """
        try:
            self.module_id = self.__thrift_client.FunctionModuleCreate("zFunctModRegister3d", "3DReg")
            success = True
            message = "Function module created"

        except Exception as e:
            success = False
            message = e
        
        return self.module_id,success,message

    def start_project(self, coord_sys):
        """Start projection on the surface of all figures (shapes) that belong to the active coordinate system.
            
        Args:
            coord_sys (str): name of the current coordinate system

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        try:
            if not coord_sys:
                success = False
                message = "None Coordinate System set"
            else:
                ref_obj_name = "RefObj_" + coord_sys
                ref_obj = self.__thrift_client.GetGeoTreeElement(ref_obj_name)

                geo_tree_list = self.__thrift_client.GetGeoTreeIds()
                matches = [geo_tree_id.name for geo_tree_id in geo_tree_list if geo_tree_id.elemType == 1024]
                polylines = [self.__thrift_client.GetProjectionElement(name) for name in matches]
                polylines_actives = ([proj_elem for proj_elem in polylines 
                                                    if proj_elem.activated == True and proj_elem.coordinateSystemList[0] == coord_sys])

                if ref_obj.activated == False or not polylines_actives:
                    success = False
                    message = "Coordinate_system is not activated or nothing to project"
                else:
                    self.__thrift_client.TriggerProjection()
                    success = True
                    message = "Projecting elements from [" + coord_sys + "] coordinate system."
        
        except Exception as e:
            success = False
            message = e
        
        return success,message

    def stop_project(self): 
        """Stop projection of all figures.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
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
            object: matrix struct initialized with empty values
        """
        mat = thrift_interface.Matrix4x4(list())
        return mat

    def create_2d_point(self, x=0, y=0):
        """Initialize 2-dimension array.
            
        Args:
            x (float): x position value
            y (float): y position value

        Returns:
            object: struct with the values of the 2 axis (x,y)
        """
        return thrift_interface.Vector2D(x, y)

    def create_3d_point(self, x=0, y=0, z=0):
        """Initialize 3-dimension array.
            
        Args:
            x (float): x position value
            y (float): y position value
            z (float): z position value

        Returns:
            object: struct with the values of the 3 axis (x,y,z)
        """
        return thrift_interface.Vector3D(x, y, z)

class CoordinateSystem(object):
    """This class implement the functions related with coordinate systems management."""
    
    def __init__(self, projector_id, module_id, thrift_client):
        """Initialize the CoordinateSystem object.
        
        Args:
            projector_id (str): serial number of the projector
            module_id (str): function module identification name
            thrift_client (object): object with the generated client to communicate with the projector
        """
        self.__thrift_client = thrift_client 
        self.__geometry_tool = GeometryTool()

        self.projector_id = projector_id
        self.module_id = module_id
        self.reference_object_list = []

    def coordinate_system_list(self):
        """Get list of current available coordinate systems from projector.

        Returns:
            tuple[list, bool, str]: the first value in the returned tuple is a names' list of available coordinate systems, 
            the second is a bool success value and the third value in the tuple is an information message string
        """
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
            object: reference object struct fields initialized
        """
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
            name (str): reference point name
            x (float): x position value
            y (float): y position value
            z (float): z position value
            active (bool): activate reference point parameter
        
        Returns:
            object: reference point struct fields initialized
        """
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
            cs (object): struct with the necessary parameters to create the coordinate system 

        Returns:
            tuple[str, bool, str]: the first value in the returned tuple is the name string of the coordinate system generated, 
            the second is a bool success value and the third value in the tuple is an information message string
        """
        try:
            reference_object = self.create_reference_object()
            reference_object.name = "RefObj_" + cs.name
            reference_object.coordinateSystem = cs.name
            reference_object.projectorID = self.projector_id
            
            resolution = cs.resolution
            T1_x = cs.T1_x
            T1_y = cs.T1_y
            T2_x = T1_x + resolution
            T2_y = 0
            T3_x = T1_x + resolution
            T3_y = T1_y + resolution
            T4_x = 0
            T4_y = T1_y + resolution
            T = [T1_x, T1_y, T2_x, T2_y, T3_x, T3_y, T4_x, T4_y]

            rot_angle = 180/math.pi*math.atan2((cs.y1 - cs.y2),(cs.x1 - cs.x2))
            print('Reference system rotation angle: {}'.format(rot_angle))
            
            # rot_matrix = np.array([[ math.cos(rot_angle), -math.sin(rot_angle)], 
            #                        [ math.sin(rot_angle),  math.cos(rot_angle)]])
            # rot_inverse = np.linalg.inv(rot_matrix)
            # T2 = np.array([[T2_x],[T2_y]]), rotated = rot_inverse.dot(T2), T2_x = float(rotated[0]), T2_y = float(rotated[1])

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

            self.__ref_obj_state(False,reference_object)

            success = True
            message = "Cordinate system defined correctly"

        except Exception as e:
            success = False 
            message = e

        return cs.name,T,success,message

    def __define_reference_point(self,reference_object,cross_size,n,d,x,y):
        """Fill other fields of the coordinate system structure.

        Args:
            reference_object (object): current coordinate system struct
            cross_size (object): struct with the dimensions of the crosses
            n (int): vector index
            d (float): distance between the projection surface and the projector
            x (float): value of the x-axis coordinate
            y (float): value of the y-axis coordinate
        
        Returns:
            object: coordinate system parameters structure updated
        """
        reference_object.refPointList[n].tracePoint.x = x
        reference_object.refPointList[n].tracePoint.y = y
        reference_object.refPointList[n].distance = d
        reference_object.refPointList[n].activated = True
        reference_object.refPointList[n].crossSize = cross_size
        return reference_object

    def __ref_obj_state(self, state, ref_obj):
        """Activate or deactivate a reference object.

        Args:
            state (bool): activation/deactivation parameter
            reference_object (object): reference object to be activated/deactivated
        """
        ref_obj.activated = state
        self.__thrift_client.SetReferenceobject(ref_obj)
        self.__thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", ref_obj.name)
        self.__thrift_client.FunctionModuleSetProperty(self.module_id, "runMode", "1")
        self.__thrift_client.FunctionModuleRun(self.module_id)

    def register_cs(self,coord_sys):
        """Register the new coordinate system at the projector once it has been generated and activated.

        Args:
            coord_sys (str): name of the coordinate system

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
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

    def set_cs(self,coord_sys): 
        """Activate the new generated coordinate system and deactivate the other existing coordinate systems at the projector.

        Args:
            coord_sys (str): name of the new coordinate system
        
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        try:
            geo_tree_list = self.__thrift_client.GetGeoTreeIds()
            matches = [geo_tree_id.name for geo_tree_id in geo_tree_list if geo_tree_id.elemType == 4096]
            coordinate_systems = [self.__thrift_client.GetReferenceobject(name) for name in matches]
            
            [self.__ref_obj_state(False, cs) for cs in coordinate_systems]
            ref_obj_name = "RefObj_" + coord_sys
            [self.__ref_obj_state(True, cs) for cs in coordinate_systems if cs.name == ref_obj_name]

            success = True
            message = coord_sys + " set as current coordinate system"

        except Exception as e:
            success = False 
            message = e

        return success,message

    def show_cs(self,coord_sys,secs):
        """Project a coordinate system on the projection surface.

        Args:
            coord_sys (str): name of the coordinate system
            secs (int): number of seconds the projection lasts

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        try:
            self.__thrift_client.FunctionModuleSetProperty(self.module_id,"showAllRefPts","1")
            time.sleep(secs)
            self.__thrift_client.FunctionModuleSetProperty(self.module_id,"showAllRefPts","0")
            
            success = True
            message = ("Finished to show [{}] coordinate system".format(coord_sys))
        
        except Exception as e:
            success = False 
            message = e

        return success,message

    def remove_cs(self,coord_sys):
        """Delete a coordinate system.
            
        Args:
            coord_sys (str): name of the coordinate system
                
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        try:
            reference_object_name = "RefObj_" + coord_sys
            self.__thrift_client.RemoveGeoTreeElem(reference_object_name)
            success = True
            message = "Coordinate system [" + coord_sys + "] removed. Set other coordinate system or define a new one before continue."

        except Exception as e:
            success = False 
            message = e

        return success,message

class ProjectionElementControl(object):
    """This class implement the functions related with figures projection."""
    
    def __init__(self,module_id, thrift_client):
        """Initialize the ProjectionElementControl object.
        
        Args:
            module_id (str): function module identification name
            thrift_client (object): object with the generated client to communicate with the projector
        """
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
            elem (object): projection element struct to initialize
            
        Returns:
            object: projection element struct with fields initialized
        """
        elem.coordinateSystemList = copy.deepcopy(self.default_projection_element.coordinateSystemList)
        elem.projectorIDList = copy.deepcopy(self.default_projection_element.projectorIDList)
        elem.userTrans = copy.deepcopy(self.default_projection_element.userTrans)
        elem.activated = self.default_projection_element.activated
        elem.pen = self.default_projection_element.pen
        return elem

    def create_polyline(self, name):
        """Generate a new line object.
            
        Args:
            name (str): line name
        
        Returns:
            object: polyline struct with fields initialized
        """
        polyline = thrift_interface.Polyline()
        polyline = self.init_projection_element(polyline)
        polyline.name = name
        polyline.polylineList = []
        return polyline

    def define_polyline(self,coord_sys,proj_elem_params):
        """Create a new line as projection figure.

        Args:
            coord_sys (str): name of coordinate system which the new projection figure will be added
            proj_elem_params (list): list with the necessary parameters to identify and define a line as a new projection figure
                
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        try:
            projection_group = proj_elem_params.projection_group_name
            shape_id         = proj_elem_params.shape_id
            x                = proj_elem_params.x
            y                = proj_elem_params.y
            angle            = proj_elem_params.angle
            length           = proj_elem_params.length

            polyline_name = projection_group + "/my_polyline_" + shape_id
            polyline = self.create_polyline(polyline_name)

            linestring = [ self.__geometry_tool.create_3d_point(x, y),
                           self.__geometry_tool.create_3d_point(x+length*math.cos(angle*math.pi/180), y+length*math.sin(angle*math.pi/180))]
            
            polyline.polylineList = [linestring]
            polyline.activated = True
            polyline.coordinateSystemList = [coord_sys]
        
            self.__thrift_client.SetPolyLine(polyline)
            
            success = True
            message = polyline_name + " polyline created at [" + coord_sys + "] coordinate system."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def deactivate_shape(self, shape_params): 
        """Hide (deactivate) a projection figure from the current reference system.

        Args:
            shape_params (list): list with the necessary parameters to identify the projection figure
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        try:
            shape_type       = shape_params.shape_type
            projection_group = shape_params.projection_group_name
            shape_id         = shape_params.shape_id

            if shape_type == "polyline":
                name = projection_group + "/my_" + shape_type + "_" + shape_id
                polyline = self.__thrift_client.GetPolyLine(name)
                if polyline:
                    polyline.activated = False
                    self.__thrift_client.SetPolyLine(polyline)
                    success = True
                    message = "Polyline " + name + " deactivated."
                else:
                    success = False
                    message = "Polyline " + name + " does not exist."
            else:
                success = False
                message = "Shape name does not exist."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def reactivate_shape(self, shape_params): 
        """Unhide (activate hidden) a projection figure from the current reference system.

        Args:
            shape_params (list): list with the necessary parameters to identify the projection figure
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        try:
            shape_type       = shape_params.shape_type
            projection_group = shape_params.projection_group_name
            shape_id         = shape_params.shape_id

            if shape_type == "polyline":
                name = projection_group + "/my_" + shape_type + "_" + shape_id
                polyline = self.__thrift_client.GetPolyLine(name)
                if polyline:
                    polyline.activated = True
                    self.__thrift_client.SetPolyLine(polyline)
                    success = True
                    message = "Polyline " + name + " reactivated."
                else:
                    success = False
                    message = "Shape name does not exist."
            else:
                success = False
                message = "Shape name does not exist."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def delete_shape(self, shape_params):
        """Delete a projection figure from the current reference system.

        Args:
            shape_params (list): list with the necessary parameters to identify the projection figure
            
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an information 
            message string
        """
        try:
            shape_type       = shape_params.shape_type
            projection_group = shape_params.projection_group_name
            shape_id         = shape_params.shape_id

            name = projection_group + "/my_" + shape_type + "_" + shape_id
            shape = self.__thrift_client.GetPolyLine(name)
            if shape:
                self.__thrift_client.RemoveGeoTreeElem(name)
                success = True
                message = "Shape removed"
            else:
                success = False
                message = "Shape name does not exist."
        except Exception as e:
            success = False 
            message = e

        return success,message
