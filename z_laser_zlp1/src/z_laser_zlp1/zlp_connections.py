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

"""Helper module for python thrift interface to ZLP Service. 
This module contains utility classes and methods which ease the usage of the thrift interface to 
communicate with the ZLP Service."""

import os
import sys
import time
import socket
import copy
import threading 

import thriftpy
from thriftpy.protocol import TBinaryProtocolFactory
from thriftpy.server import TThreadedServer, TSimpleServer
from thriftpy.thrift import TProcessor, TClient
from thriftpy.transport import TBufferedTransportFactory, TServerSocket, TSocket

class EventChannelInterfaceHandler(object):
    """This class implement the functions of ClientEventChannel thrift interface.

    Attributes:
        property_changed_callback (object): callback to handle settings changes on laser projector system
        geo_tree_changed_callback (object): callback to handle changes on geotree operator
        service_state_changed_callback (object): callback to handle changes on services state
        function_module_changed_callback (object): callback to handle changes on function module state
        rc_command_received_callback (object): callback to handle remote control commands reception
        on_reflection_state_changed_callback (object): callback to handle changes on reflection state
    """
    def __init__(self):
        """Initialize the EventChannelInterfaceHandler object."""
        self.property_changed_callback = lambda x, y: None
        self.geo_tree_changed_callback = lambda x, y: None
        self.service_state_changed_callback = lambda x, y: None
        self.function_module_changed_callback = lambda x, y, z: None
        self.rc_command_received_callback = lambda a, b, c, d, e: None
        self.on_reflection_state_changed_callback = lambda a, b: None

    def PropertyChanged(self, name, value):
        """Set callback function to handle settings changes on laser projector system.
    
        Args:
            name (str): full path of property that was changed
            value (int): value of property 
        """
        self.property_changed_callback(name, value)

    def GeoTreeChanged(self, changed_flags, element_names):
        """Set callback function to handle changes on geotree operator.
    
        Args:
            changed_flags (int): integer value with flags of type GeoTreeChangedFlags
            element_names (enum): identification of changed element (within the GeoTreeElemId enumeration )
        """
        self.geo_tree_changed_callback(changed_flags, element_names)

    def ServiceStateChanged(self, oldState, newState):
        """Set callback function to handle changes on services state.
    
        Args:
            oldState (enum): old state (within the ServiceStates enumeration) before change 
            newState (enum): new state (within the ServiceStates enumeration) after change 
        """
        self.service_state_changed_callback(oldState, newState)

    def FunctionModuleStateChanged(self, functionModID, oldState, newState):
        """Set callback function to handle changes on function module state.
    
        Args:
            functionModID (str): identificator name of function module
            oldState (enum): old state (within the FunctionModuleStates enumeration) before change
            newState (enum): new state (within the FunctionModuleStates enumeration) after change
        """
        self.function_module_changed_callback(functionModID, oldState, newState)

    def RemoteControlFrameReceived(self, rc_id, command, toggle, projector, timestamp):
        """Set callback function to handle remote control commands reception.
    
        Args:
            rc_id (str): address of RC-device 
            command (enum): enum with command codes for remotecontrol functions 
            toggle (bool): toggle function active
            projector (str): serial number of the projector
            timestamp (int): timestamp
        """
        self.rc_command_received_callback(rc_id, command, toggle, projector, timestamp)

    def onReflectionStateChanged(self, elementName, state):
        """Set callback function to handle changes on reflection state.
    
        Args:
            elementName (str): name of the element that changed state 
            state (bool): true if a reflection was detected; False otherwise
        """
        self.on_reflection_state_changed_callback(elementName, state)

class ThriftClient(TClient):
    """This class implement the functions to carry out the connection with the ZLP Service.
    
    Args:
        event_handler (object): object with functions of ClientEventChannel thrift interface

    Attributes:
        thrift_interface (obj): load the interface description file (interface.thrift) for the communication between 
        ZLP-Service and a remote client
    """
    def __init__(self, event_handler=EventChannelInterfaceHandler()):
        """Initialize the ThriftClient object."""
        self._event_channel = None
        self._event_channel_handler = event_handler

        _interface_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "interface.thrift")
        self.thrift_interface = thriftpy.load(_interface_file, module_name="zlaser_thrift")

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
        super().__init__(self.thrift_interface.ServiceInterface, protocol)

    def init_event_channel(self):
        """Create a thrift server and register it at ZLP Service to receive events."""
        if self._event_channel_handler and not self._event_channel:
            processor = TProcessor(self.thrift_interface.ClientEventChannel, self._event_channel_handler)
            server_socket = TServerSocket(host="0.0.0.0", port=0, socket_family=socket.AF_INET, client_timeout=200000)
            server_socket.client_timeout = 1000*60*10 
            self._event_channel = TSimpleServer(processor, server_socket)

            t = threading.Thread(target=self._event_channel.serve, daemon=True)
            t.start()

            time.sleep(1)
            connection = self._event_channel.trans.sock.getsockname()
            self.ConnectClientEventChannel(connection[1])

    def set_property_changed_callback(self, callback):
        """Set callback function related with laser projector settings changes.
            
        Args:
            callback (object): callback function to set

        Raises:
            ValueError
        """
        if self._event_channel_handler:
            self._event_channel_handler.property_changed_callback = callback
        else:
            raise ValueError("Error: Can't install callback, because event_handler = none!")

    def set_geotree_changed_callback(self, callback):
        """Set callback function related with geotree operator changes.
            
        Args:
            callback (object): callback function to set

        Raises:
            ValueError
        """
        if self._event_channel_handler:
            self._event_channel_handler.geo_tree_changed_callback = callback
        else:
            raise ValueError("Error: Can't install callback, because event_handler = none!")

    def set_function_module_state_changed_callback(self, callback):
        """Set callback function related with function module state changes.
            
        Args:
            callback (object): callback function to set

        Raises:
            ValueError
        """
        if self._event_channel_handler:
            self._event_channel_handler.function_module_changed_callback = callback
        else:
            raise ValueError("Error: Can't install callback, because event_handler = none!")

    def set_rc_command_received_callback(self, callback):
        """Set callback function related with remote control commands reception.
            
        Args:
            callback (object): callback function to set

        Raises:
            ValueError
        """
        if self._event_channel_handler:
            self._event_channel_handler.rc_command_received_callback = callback
        else:
            raise ValueError("Error: Can't install callback, because event_handler = none!")

    def set_reflection_state_changed_callback(self, callback):
        """Set callback function related with reflection state changes.
            
        Args:
            callback (object): callback function to set

        Raises:
            ValueError
        """
        if self._event_channel_handler:
            self._event_channel_handler.on_reflection_state_changed_callback = callback
        else:
            raise ValueError("Error: Can't install callback, because event_handler = none!")

class ProjectorClient(object):
    """This class implements the functions for connecting to the projector and basic projection features.

    Attributes:
        projector_id (str): serial number of the projector
        module_id (str): function module identification name
    """
    def __init__(self):
        """Initialize the ProjectorClient object."""
        self.projector_id = ""
        self.module_id = ""

        self.__thrift_client = ThriftClient()

        self.cv = threading.Condition()

    def get_thrift_client(self):
        """Return the object generated to communicate with the projector.
            
        Returns:
            object: thrift client object generated to communicate with the projector
        """
        try:
            return self.__thrift_client
        
        except Exception as e:
            return e

    def connect(self,server_IP,connection_port):
        """Create and connect the client to thrift server (located at projector) of ZLP-Service and establish an event channel if 
        needed.

        Args:
            server_IP (str): ipv6 network address of ZLP-Service
            connection_port (str): port number on which ZLP-Service listens for requests

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            if not self.__thrift_client._event_channel:
                self.__thrift_client.init_client(server_IP, connection_port)
                self.__thrift_client.init_event_channel()
                success = True 
                message = "Client connected"
            else:
                success = False 
                message = "Projector already connected"

        except Exception as e:
            success = False 
            message = e
    
        return success,message

    def disconnect(self):
        """Disconnect from ZLP Service thrift server and close own event server.
        
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try: 
            self.__thrift_client.RemoveGeoTreeElem("") 
            self.__thrift_client.FunctionModuleRelease(self.module_id)
            
            self.__thrift_client.DisconnectClientEventChannel()
            self.__thrift_client.close()

            if self.__thrift_client._event_channel:
                self.__thrift_client._event_channel.close()
                self.__thrift_client._event_channel = None

            success = True 
            message = "Projector disconnected"

        except Exception as e:
            success = False 
            message = e

        return success,message

    def connection_status(self):
        """Get status of projection connection.

        Returns:
            bool: status of the event channel object. Projector connected if true, disconnected otherwise
        """
        return self.__thrift_client._event_channel

    def transfer_license(self, lic_path):
        """Transfer data of the local license file to remote file at ZLP-Service.
        
        Args:
            lic_path (str): license file path

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            license_path = os.path.abspath(lic_path)
            license_file = os.path.basename(license_path)
            content = open(license_path, 'r').read()
            self.__thrift_client.TransferDataToFile(content, license_file, True)

            self.__thrift_client.LoadLicense(license_file)

            success = True
            message = "License transfered."

        except self.__thrift_client.thrift_interface.CantWriteFile as e:
            success = False
            message = e
        
        except FileNotFoundError as e:
            success = False
            message = e

        except Exception as e:
            success = False
            message = e

        return success,message

    def check_license(self):
        """Check if license is valid.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
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
            tuple[list, bool, str]: the first value in the returned tuple is a list of serial numbers of the projectors found, 
            the second a bool success value and the third value in the tuple is an information message string
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
        """Callback function related with laser projector settings changes.
        
        Args:
            prop (str): full path of property that was changed
            value (int): value of property 
        """
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
            projectors, success, message = self.scan_projectors(projector_IP)
            if success:
                self.__thrift_client.set_property_changed_callback(self.property_changed_callback)
                self.__thrift_client.RegisterForChangedProperty("config.licenseState.IsValid")

                self.cv.acquire()
                self.projector_id = projectors[0]
                self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector.serial", self.projector_id)
                self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector.active", "1")
                self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector", "1")
                self.cv.wait()
                self.cv.release()
                message = "Projector activated"
        
        except Exception as e:
            success = False 
            message = e
        
        return self.projector_id,success,message

    def deactivate_projector(self):
        """Set properties to deactivate a projector.
        
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
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
        """Create function module to operate with GeoTreeElements (coordinate systems and projection elements).

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

    def start_project(self, cs_name):
        """Start projection on the surface of all projection elements that belong to the active coordinate system.
            
        Args:
            cs_name (str): name of the active coordinate system

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            if not cs_name:
                success = False
                message = "None Coordinate System set"

            if not self.__thrift_client.GetGeoTreeElement(cs_name).activated:
                success = False
                message = "Coordinate_system is not activated"

            if self.is_empty(cs_name):
                success = False
                message = "Nothing to project"

            else:
                self.__thrift_client.TriggerProjection()
                success = True
                message = "Projecting elements from [" + cs_name + "] coordinate system."
        
        except Exception as e:
            success = False
            message = e
        
        return success,message

    def stop_project(self): 
        """Stop projection of all elements.

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
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

    def update_project(self,cs_name): 
        """Update changes on figures projected (restart projection).

        Args:
            cs_name (str): name of the coordinate system to update

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            self.stop_project()
            self.start_project(cs_name)
            success = True
            message = "Projection updated."
        
        except Exception as e:
            success = False
            message = e

        return success,message

    def is_empty(self, cs_name):
        """Check if coordinate system has associated projection elements.

        Args:
            cs_name (str): name of the coordinate system to check

        Returns:
            bool: true if there is any projection element defined at the coordinate system, false otherwise
        """
        is_empty = False
        try:
            geo_tree_list = self.__thrift_client.GetGeoTreeIds()
            # elemType = 1024, 1025, 1026, 1027 refers to projection element identificator at the projector device
            matches = [elem.name for elem in geo_tree_list if elem.elemType in (1024,1025,1026,1027)]
            proj_elems = [self.__thrift_client.GetProjectionElement(name) for name in matches]
            
            for proj_elem in proj_elems:
                if proj_elem.activated == True and proj_elem.coordinateSystemList[0] == cs_name:
                    proj_elems_actives = proj_elem

            if not proj_elems_actives:
                is_empty = True 
        
        except Exception:
            is_empty = True 

        return is_empty