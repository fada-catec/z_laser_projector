#! python
# -*- coding: utf-8 -*-
"""Helper module for python thrift interface to ZLP Service.

This module contains utility classes and methods which ease the usage of the thrift interface to ZLP Service.
"""

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
# Thriftpy 0.3.9 has problems with absolute Windows paths. The issue is fixed in later
# versions. See:
#   https://github.com/Thriftpy/thriftpy/issues/234
#   https://github.com/Thriftpy/thriftpy/pull/285/commits/2147b2d0cf2dccee0e7cdabf85595e6ebfded849
#
# The solution is to patch .../thriftpy/parser/parser.py to handle filesystem paths
# propperly. Without this fix an error of the form:
#   "ThriftPy does not support generating module with path in protocol 'c'"
#
# is raised here.
thrift_interface = thriftpy.load(_interface_file, module_name="zlaser_thrift")


class EventChannelInterfaceHandler(object):
    """Implements the functions of ClientEventChannel thrift interface."""

    def __init__(self):
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
        log.debug("onReflectionStateChanged(%s,%d)", elementName, state)
        self.onReflectionStateChanged_callback(elementName, state)

class ThriftClient(TClient):
    def __init__(self, event_handler=EventChannelInterfaceHandler()):
        self._event_channel = None
        self._event_channel_handler = event_handler

    def __init_client(self, ip, port):
        """Establish a connection to thrift server of ZLP Service."""
        client_socket = TSocket(ip, port, socket_family=socket.AF_INET, socket_timeout=50000)
        transport = TBufferedTransportFactory().get_transport(client_socket)
        protocol = TBinaryProtocolFactory().get_protocol(transport)
        transport.open()
        super().__init__(thrift_interface.ServiceInterface, protocol)
        log.info("Connecting to ZLP Service at %s:%s" % (ip, port))

    def __init_event_channel(self):
        """Create a thrift server and register it at ZLP Service to receive events."""
        if self._event_channel_handler and not self._event_channel:
            processor = TProcessor(thrift_interface.ClientEventChannel, self._event_channel_handler)
            server_socket = TServerSocket(host="0.0.0.0", port=0, socket_family=socket.AF_INET, client_timeout=200000)
            server_socket.client_timeout = 1000*60*10 # 10 min for timeout, because when should an event come?
            self._event_channel = TSimpleServer(processor, server_socket)

            t = Thread(target=self._event_channel.serve, daemon=True)
            t.start()

            time.sleep(1)  # Waiting for the event channel server to start up
            connection = self._event_channel.trans.sock.getsockname()
            log.info("Listening for events on %s:%s" % (connection[0], connection[1]))
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

    def __init__(self,projector_IP,server_IP,connection_port):
        self.projector_IP = projector_IP
        self.server_IP = server_IP
        self.connection_port = connection_port

        self.__thrift_client = ThriftClient()
        self.projector_id = ""
        self.module_id = ""

    def connect(self):
        """Connects the client to ZLP-Service and establishes an event channel if needed.

            Args:
                ip: ipv6 network address of ZLP-Service
                port: port number on which ZLP-Service listens for requests """
        
        self.__thrift_client.__init_client(self.server_IP, self.connection_port)
        self.__thrift_client.__init_event_channel()

    def disconnect(self):
        """Disconnect from ZLP Service thrift server and close own event server."""
        
        self.__thrift_client.DisconnectClientEventChannel() # NO PODRÍA SER: thrift_interface.DisconnectClientEventChannel() ???????????
        self.__thrift_client.close()

        if self.__thrift_client._event_channel:
            self.__thrift_client._event_channel.close()

    def get_projectors(self, scan=False, scan_addresses=""):
        """Gets a list of active projectors or scan the network for projectors.

            Args:
                scan: scan the network for projectors
                addresses: addresses or address are to scan

            Returns:
                list: serial numbers of the found projectors """
        
        try:
            # set parameters and start the search
            if scan:
                self.__thrift_client.SetProperty("config.projectorManager.cmdGetProjectors.scan", "1")
            else:
                self.__thrift_client.SetProperty("config.projectorManager.cmdGetProjectors.scan", "0")

            self.__thrift_client.SetProperty("config.projectorManager.cmdGetProjectors.scanAddresses", scan_addresses)
            self.__thrift_client.SetProperty("config.projectorManager.cmdGetProjectors", "1")

            # Get the results
            serial_list = self.__thrift_client.GetProperty("config.projectorManager.cmdGetProjectors.result.entries")

            if serial_list:
                serial_list = serial_list.split(" ")
                return serial_list

        except Exception as e:
            log.error(e)

        return []

    def scan_projectors(self, addresses=""):
        """Scan the network for projectors.

            Args:
                addresses: addresses or address are to scan

            Returns:
                list: serial numbers of the found projectors """
        
        log.info("Scanning for projectors...")
        serial_list = self.get_projectors(True, addresses)
        if (len(serial_list) == 0):
            log.warning("No projectors were found!")
            return []

        log.info("Available projectors: " + str(serial_list))
        return serial_list

    def activate_projector(self):
        try:
            projectors = self.scan_projectors(self.projector_IP)
            self.projector_id = projectors[0]
            log.info("Activating projector. ID: " + self.projector_id)
            self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector.serial", self.projector_id)
            self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector.active", "1")
            self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector", "1")
            #blocks until the projector is activated
            self.get_projectors()
            # return self.projector_id
        # except Exception as e:
        except Exception:
            log.error("Could not activate projector:" + self.projector_id)

    def deactivate_projector(self):
        try:
            log.info("Deactivating projection of projector: " + self.projector_id)
            projector_property_path = "config.projectorManager.projectors." + self.projector_id
            self.__thrift_client.SetProperty(projector_property_path + ".cmdShowProjection.show", "0")
            self.__thrift_client.SetProperty(projector_property_path + ".cmdShowProjection", "1")

            # HAY QUE DISTINGUIR ENTRE DESCONECTAR Y BORRAR TODO, O DESCONECTAR SOLO, AHORA MISMO BORRA TODO
            # QUE HACER ADEMAS AL DEACTIVATE: ????
            # self.geo_tree_elements.clear() ?? <- geo_tree_elements[] es un es una lista de geo_tree_elem creada por nosotros (.clear() elimina los elementos del vector)
            
            # thrift_client.RemoveGeoTreeElem("") <- remove all!!!!
            # thrift_client.FunctionModuleRelease(module_id) remove all tambien??

            # CUANDO SE DESENCHUFA SE BORRA TODO
            
            # if hasattr(self,'reference_object_name'):
            # self.thrift_client.RemoveGeoTreeElem(self.reference_object_name) # necesario? se pone mejor en un método aparte remove_geo_tree_elem no?
            # self.clear_geo_tree() # al hacer deactivate en el proyector se borran automaticamente los geo_trees? es mejor dejar ese if para borrarlos? no hace falta?
                # se borran al desenchufar el proyector

            log.info("Removing all projection elements")
            self.__thrift_client.RemoveGeoTreeElem("") 
            self.__thrift_client.FunctionModuleRelease(self.module_id)

            log.info("Deactivating projector: " + self.projector_id)
            self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector.serial", self.projector_id)
            self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector.active", "0")
            self.__thrift_client.SetProperty("config.projectorManager.cmdActivateProjector", "1")

        except Exception as e:
            log.error("Error: projector could not be deactivated")
            raise e

    def transfer_license(self, lic_path):
        try:
            license_path = os.path.abspath(lic_path)
            license_file = os.path.basename(license_path)
            self.transfer_file(license_path, license_file, True)
        except thrift_interface.CantWriteFile as e:
            return e
        except FileNotFoundError:
            return "File not found!"
        self.__thrift_client.LoadLicense(license_file)
        return "License transfered"

    def transfer_file(self, local_path, remote_file, overwrite=False):
        content = open(local_path, 'r').read()
        self.__thrift_client.TransferDataToFile(content, remote_file, overwrite)

    def check_license(self):
        return self.__thrift_client.CheckLicense()

    def function_module_create(self):
        try:
            self.module_id = self.__thrift_client.FunctionModuleCreate("zFunctModRegister3d", "3DReg")
            return self.module_id, self.projector_id
        except thrift_interface.FunctionModuleClassNotRegistered as e:
            return ("FunctionModuleClassNotRegistered: " + e.which)
        except thrift_interface.FunctionModulePropertyBranchAlreadyInUse as e:
            return ("FunctionModulePropertyBranchAlreadyInUse: " + e.branchName)
        except thrift_interface.FunctionModuleClassNotLicensed as e:
            return ("FunctionModuleClassNotLicensed: " + e.which)

    def start_project(self, coord_sys):
        """Start projection on the surface of all figures (shapes) that belong to the active coordinate system.
            
            Returns:
                string: message """
        
        # self.thrift_client.TriggerProjection()
        # return(" ----- PROJECTING ----- ")

        ref_obj_name = "RefObj_" + coord_sys
        ref_obj = self.__thrift_client.GetGeoTreeElement(ref_obj_name)
        # GetGeoTreeIds() saca tanto coord sys (son los primeros de la lista) como los shapes, mientras que GetCoordinatesystemList solo saca los cs
        if ref_obj.activated == False or len(self.__thrift_client.GetGeoTreeIds()) <= len(self.__thrift_client.GetCoordinatesystemList()):                                        
            return(" ----- NOTHING TO PROJECT ----- ")
        else:
            self.__thrift_client.TriggerProjection()
            return(" ----- PROJECTING ----- ")

    def stop_project(self): # este tipo de método son los que se incluyen en zlp pero este concretamente no está incluido, se añade aqui para no tocar zlp
        """Stop projection of all figures.
            
            Returns:
                string: message """
        
        try:
            projector_property_path = "config.projectorManager.projectors." + self.projector_id
            self.__thrift_client.SetProperty(projector_property_path + ".cmdShowProjection.show", "0")
            self.__thrift_client.SetProperty(projector_property_path + ".cmdShowProjection", "1")
            return(" ----- STOP PROJECTION ----- ")
        except Exception as e:
            return e


class GeometryTool():

    def create_matrix4x4(self):
        mat = thrift_interface.Matrix4x4(list())
        return mat

    def init_element_3d(self, elem):
        elem.userTrans = self.create_matrix4x4()
        return elem

    def create_2d_point(self, x=0, y=0):
        return thrift_interface.Vector2D(x, y)

    def create_3d_point(self, x=0, y=0, z=0):
        return thrift_interface.Vector3D(x, y, z)

class CoordSys(object):

    def __init__(self, projector_id, module_id):
        self.__thrift_client = ThriftClient()
        self.__geometry_tool = GeometryTool()

        self.projector_id = projector_id
        self.module_id = module_id
        # self.current_cs = ""
        self.reference_object_list = []

    def coordinate_system_list(self):
        return self.__thrift_client.GetCoordinatesystemList()
        # allGeoTree = self.thrift_client.GetGeoTreeIds()
        # print("Available GeoTree:", allGeoTree)

    def create_reference_object(self):
        ref_obj = thrift_interface.Referenceobject()
        ref_obj.name = ""
        ref_obj.activated = False
        ref_obj.fieldTransMat = self.__geometry_tool.create_matrix4x4()
        ref_obj.refPointList = []
        ref_obj.projectorID = ""
        ref_obj.coordinateSystem = ""
        return ref_obj

    def create_reference_point(self, name, x, y, z=0, active=True):
        ref_point = thrift_interface.Referencepoint()
        ref_point.name = name
        ref_point.refPoint = self.__geometry_tool.create_3d_point(x, y, z)
        ref_point.activated = active
        ref_point.tracePoint = self.__geometry_tool.create_2d_point()
        ref_point.crossSize = self.__geometry_tool.create_2d_point()
        ref_point.distance = 0
        return ref_point

    def define_cs(self,req):
        """Generate new coordinate system.

            Args:
                struct (req): structure with the necessary parameters value to generate the coordinate system
            
            Returns:
                string: name of the coordinate system generated """
        
        reference_object = self.create_reference_object()
        reference_object.name = "RefObj_" + req.name_cs.data
        print("Creating reference object: {}".format(reference_object.name))
        reference_object.coordinateSystem = req.name_cs.data
        reference_object.projectorID = self.projector_id

        T2_x = req.T1_x.data + abs((req.x2.data - req.x1.data))
        T2_y = req.T1_y.data
        T3_x = req.T1_x.data + abs((req.x3.data - req.x1.data))
        T3_y = req.T1_y.data + abs((req.y3.data - req.y1.data))
        T4_x = req.T1_x.data
        T4_y = req.T1_y.data + abs((req.y4.data - req.y1.data))

        # scale_factor = ??? T_x,y * 2 # manteniendo la proporción

        reference_object.refPointList = [   self.create_reference_point("T1", req.T1_x.data, req.T1_y.data),
                                            self.create_reference_point("T2",     T2_x,          T2_y),
                                            self.create_reference_point("T3",     T3_x,          T3_y),
                                            self.create_reference_point("T4",     T4_x,          T4_y)]
        
        crossSize = self.__geometry_tool.create_2d_point(req.crossize_x.data,req.crossize_y.data) # set global crosssize for all reference points

        reference_object = self.__define_reference_point(reference_object,crossSize,0,req.distance.data,req.x1.data,req.y1.data) # define coordinates in user system [mm]
        reference_object = self.__define_reference_point(reference_object,crossSize,1,req.distance.data,req.x2.data,req.y2.data)
        reference_object = self.__define_reference_point(reference_object,crossSize,2,req.distance.data,req.x3.data,req.y3.data)
        reference_object = self.__define_reference_point(reference_object,crossSize,3,req.distance.data,req.x4.data,req.y4.data)

        # reference_object.activated = True # en set_cs
        # self.thrift_client.SetReferenceobject(reference_object) # en set_cs
        # self.thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", reference_object.name) # en set_cs
        
        self.add_ref_object(reference_object) 

        # print("Reference object created. Coordinate system defined but not registered.")
        return reference_object.coordinateSystem

    def __define_reference_point(self,reference_object,crossSize,n,d,x,y):
        """Fill other fields of the coordinate system parameters structure.

            Args:
                struct (reference_object): current coordinate system parameters structure
                struct (crossSize): struct with the dimensions of the cross
                int (n): index of the vector
                d (float): distance value between the projection surface and the projector
                x (float): value of the x-axis coordinate
                y (float): value of the y-axis coordinate
            
            Returns:
                struct: coordinate system parameters structure updated """
        
        reference_object.refPointList[n].tracePoint.x = x
        reference_object.refPointList[n].tracePoint.y = y
        reference_object.refPointList[n].distance = d
        reference_object.refPointList[n].activated = True
        reference_object.refPointList[n].crossSize = crossSize
        return reference_object

    def add_ref_object(self,ref_obj):
        """Add new coordinate system to the list.

            Args:
                struct (reference_object): parameters structure of the generated coordinate system """
        
        self.reference_object_list.append(ref_obj)
        print("[{}] appended".format(self.reference_object_list[-1].name))
        # print(self.reference_object_list[:])
        # print("Reference object list: [{}]".format(self.reference_object_list[:].name))
        # print("Reference object list: [{}]".format(self.reference_object_list))
        # print(type(self.reference_object_list))

    def set_cs(self,coord_sys): 
        """Activate the new generated coordinate system and deactivate the other existing coordinate systems at the projector.

            Args:
                string (coord_sys): name of the new coordinate system
            
            Returns:
                string: message """
        
        # SI SE HACE DISCONNECT EN EL PROYECTOR SE PIERDE LA INFO DE LOS REF_OBJECTS -> hay que eliminarlos todos en el disconnect
        # porque se borra la info de los ref_obj pero no los nombres de los coord sys??
        
        ref_obj_name = "RefObj_" + coord_sys 

        for i in range (0,len(self.reference_object_list)):
            if self.reference_object_list[i].name == ref_obj_name:
                index = i
            else:
                print("{} DEACTIVATED" .format(self.reference_object_list[i].name))
                self.reference_object_list[i].activated = False
                self.__thrift_client.SetReferenceobject(self.reference_object_list[i])
                self.__thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", self.reference_object_list[i].name)
                self.__thrift_client.FunctionModuleSetProperty(self.module_id, "runMode", "1")
                self.__thrift_client.FunctionModuleRun(self.module_id)
        
        print("{} ACTIVATED" .format(self.reference_object_list[index].name))
        self.reference_object_list[index].activated = True
        self.__thrift_client.SetReferenceobject(self.reference_object_list[index])
        self.__thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", self.reference_object_list[index].name)
        self.__thrift_client.FunctionModuleSetProperty(self.module_id, "runMode", "1")
        self.__thrift_client.FunctionModuleRun(self.module_id)

        # self.current_cs = coord_sys # set the object.coordinate_system property value to use it wherever - HACE FALTA???

        # allReferenceObjects = self.thrift_client.GetGeoTreeIds()
        # print("Available reference objects:", allReferenceObjects)

        # ref_obj = self.thrift_client.GetReferenceobject("RefObject1")
        # print(ref_obj)

        return ("[{}] set as current coordinate system".format(coord_sys))

    def register_cs(self,coord_sys):
        """Register the new coordinate system at the projector once it has been generated and activated.

            Args:
                string (coord_sys): name of the coordinate system
            
            Returns:
                string: message """
        
        print("Registering coordinate system {}".format(coord_sys))

        self.__thrift_client.FunctionModuleSetProperty(self.module_id, "runMode", "1")
        
        self.__thrift_client.FunctionModuleRun(self.module_id) # Calculate transformation

        state = self.__thrift_client.FunctionModuleGetProperty(self.module_id, "state")
        if state != "1":  # idle
            return "Function module is not in idle state, hence an error has occured."
        else:
            return ("Finished to register [{}] coordinate system on projector".format(coord_sys))

    def show_cs(self,coord_sys,secs):
        """Project on the surface an existing coordinate system.

            Args:
                string (coord_sys): name of the coordinate system
                int (secs): number of seconds the projection lasts

            Returns:
                string: message """
        
        print("Projecting [{}] coordinate system for {} seconds".format(coord_sys,secs))
        self.__thrift_client.FunctionModuleSetProperty(self.module_id,"showAllRefPts","1")
        time.sleep(secs) # input("PROJECTING COORDINATE SYSTEM ORIGIN AXES. PRESS ENTER TO FINISH.") # Y ASI PUEDO QUITAR EL SLEEP Y UNIFICAR OS SERVICIOS ShowCs y RemovCs en uno Solo
        self.__thrift_client.FunctionModuleSetProperty(self.module_id,"showAllRefPts","0")
        return "Finished to show coordinate system"

    def remove_cs(self,coord_sys):
        """Delete a coordinate system.
            Args:
                string (coord_sys): name of the coordinate system
                
            Returns:
                string: message """
        
        # for name in self.geo_tree_elements:
            # self.thrift_client.RemoveGeoTreeElem(name)
        #self.geo_tree_elements.clear()

        reference_object_name = "RefObj_" + coord_sys
        self.__thrift_client.RemoveGeoTreeElem(reference_object_name)
        return("Coordinate system [{}] removed".format(coord_sys))


class ProjectionElement(object):
    
    def __init__(self,module_id):
        self.__thrift_client = ThriftClient()
        self.__geometry_tool = GeometryTool()
        self.module_id = module_id

        # use the default elements to set default values for all new projection elements
        self.default_projection_element = thrift_interface.ProjectionElement()
        self.default_projection_element.pen = 0
        self.default_projection_element.coordinateSystemList = []
        self.default_projection_element.projectorIDList = []
        self.default_projection_element.userTrans = self.__geometry_tool.create_matrix4x4()
        self.default_projection_element.activated = True

    def init_projection_element(self, elem):
        elem.coordinateSystemList = copy.deepcopy(self.default_projection_element.coordinateSystemList)
        elem.projectorIDList = copy.deepcopy(self.default_projection_element.projectorIDList)
        elem.userTrans = copy.deepcopy(self.default_projection_element.userTrans)
        elem.activated = self.default_projection_element.activated
        elem.pen = self.default_projection_element.pen
        # `elem.detectReflection` has a default value
        return elem

    def create_projection_element(self, name):
        projection_element = copy.deepcopy(self.default_projection_element)
        projection_element.name = name
        return projection_element

    def create_polyline(self, name):
        polyline = thrift_interface.Polyline()
        polyline = self.init_projection_element(polyline)
        polyline.name = name
        polyline.polylineList = []
        return polyline

    def define_polyline(self,coord_sys,projection_group,id,x,y,angle,r):
        """Create a new line to project.

            Args:
                string (projection_group): name of the projection group
                string (id): name of the shape identificator
                float (x): x-axis value of the initial point position 
                float (y): y-axis value of the initial point position 
                float (angle): line angle value
                float (r): length of the line
                int (secs): number of seconds the projection lasts
                
            Returns:
                string: message """
        
        polyline_name = projection_group + "/my_polyline_" + id
        polyline = self.create_polyline(polyline_name)
        # self.geo_tree_elements.append(name)

        linestring = [ self.__geometry_tool.create_3d_point(x, y),
                       self.__geometry_tool.create_3d_point(x+r*math.cos(angle*math.pi/180), y+r*math.sin(angle*math.pi/180))]
        
        polyline.polylineList = [linestring]
        polyline.activated = True
        polyline.coordinateSystemList = [coord_sys]
        try:
            self.__thrift_client.SetPolyLine(polyline)
            return ("{} polyline created ".format(polyline_name))
        except Exception as e:
            return e

    # def create_circle(self,projection_group,id,x,y,r):
    #     circle_name = projection_group + "/my_circle_" + id
    #     circle = zlp.create_circle(x,y,r,circle_name)

    #     circle.activated = True
    #     circle.coordinateSystemList = self.current_cs
    #     try:
    #         self.thrift_client.SetCircleSegment(circle)
    #         return "Defined a circle segment to project"
    #     except Exception as e:
    #         return e

    def deactivate_shape(self,projection_group,shape_name,id): # deactivate shape
        """Hide (deactivate) a figure from a group of the active coordinate system.

            Args:
                string (projection_group): name of the projection group
                string (shape_name): type of figure (polyline, circle, etc.)
                string (id): name of the shape identificator
                
            Returns:
                string: message """
        
        if shape_name == "polyline":
            name = projection_group + "/my_" + shape_name + "_" + id
            polyline = self.__thrift_client.GetPolyLine(name)
            polyline.activated = False
            self.__thrift_client.SetPolyLine(polyline)
            return ("Polyline {} deactivated".format(name))
        else:
            return "Shape name does not exist"
    
        # # reference_object.activated = False  # <- puede servir 

        # # name = projection_group + "/my_" + shape_name + "_" + id
        # # shape = self.thrift_client.GetProjectionElement(name)
        # # shape.activated = False
        # # if shape_name == "polyline":
        # #     self.thrift_client.SetPolyLine(polyline)

    def reactivate_shape(self,projection_group,shape_name,id): # deactivate shape
        """Unhide (activate) a figure from a group of the active coordinate system.

            Args:
                string (projection_group): name of the projection group
                string (shape_name): type of figure (polyline, circle, etc.)
                string (id): name of the shape identificator
                
            Returns:
                string: message """
        
        if shape_name == "polyline":
            name = projection_group + "/my_" + shape_name + "_" + id
            polyline = self.__thrift_client.GetPolyLine(name)
            polyline.activated = True
            self.__thrift_client.SetPolyLine(polyline)
            return ("Polyline {} reactivated".format(name))
        else:
            return "Shape name does not exist"

    def delete_shape(self,projection_group,shape_name,id):
        """Delete a figure from the active coordinate system.

            Args:
                string (projection_group): name of the projection group
                string (shape_name): type of figure (polyline, circle, etc.)
                string (id): name of the shape identificator
                
            Returns:
                string: message """
        
        self.__thrift_client.RemoveGeoTreeElem(projection_group + "/my_" + shape_name + "_" + id)
        return "Shape removed"

    # def remove_group(self,projection_group):
    #     self.thrift_client.RemoveGeoTreeElem(projection_group)

    # def create_circle(self, x, y, r, name):
    #     circle = thrift_interface.CircleSegment()
    #     circle = self.init_projection_element(circle)
    #     circle.name = name
    #     circle.radius = r
    #     circle.center = self.create_3d_point(x, y)
    #     return circle

    # def create_oval(self, x, y, w, h, name, angle=0):
    #     oval = thrift_interface.OvalSegment()
    #     oval = self.init_projection_element(oval)
    #     oval.name = name
    #     oval.width = w
    #     oval.height = h
    #     oval.angle = angle
    #     oval.center = self.create_3d_point(x, y)
    #     return oval

    # def create_text_element(self, x, y, text, name, height):
    #     textelem = thrift_interface.TextElement()
    #     textelem = self.init_projection_element(textelem)
    #     textelem.name = name
    #     textelem.position = self.create_3d_point(x, y)
    #     textelem.text = text
    #     textelem.height = height
    #     return textelem

    # def create_driftcompensation_object(self, name="", active=True):
    #     dc_obj = thrift_interface.DriftCompensationObject()
    #     dc_obj.name = name
    #     dc_obj.activated = active
    #     dc_obj.compensationTransMat = self.create_matrix4x4()
    #     dc_obj.dcPointList = []
    #     dc_obj.projectorID = ""
    #     dc_obj.fixedTranslation = [False,False,False]
    #     dc_obj.fixedRotation = [False,False,False]
    #     return dc_obj

    # def create_driftcompensation_point(self, name, x=0, y=0, z=0, distanceToOrigin=0, active=True):
    #     dc_point = thrift_interface.DriftCompensationPoint()
    #     dc_point.name = name
    #     dc_point.activated = active
    #     dc_point.isReserve = False
    #     dc_point.traceVector = self.create_3d_point(x, y, z)
    #     dc_point.crossSize = 100
    #     dc_point.distanceToOrigin = distanceToOrigin
    #     dc_point.weight = 1
    #     return dc_point

    # def create_clip_group(self, name, type=thrift_interface.ClipSetTypes.Union, active=True):
    #     clip_group = thrift_interface.ClipGroup()
    #     clip_group.name = name
    #     clip_group.projectorMap = {}
    #     clip_group.coordinateSystem = ""
    #     clip_group.setType = type
    #     clip_group.activated = active
    #     return clip_group

    # def create_clip_plane(self, name, x, y, z, normal, coordsys, active=True):
    #     clip_plane = thrift_interface.ClipPlane()
    #     clip_plane.name = name
    #     clip_plane.point = self.create_3d_point(x, y, z)
    #     clip_plane.normal = normal
    #     clip_plane.activated = active
    #     clip_plane.coordinateSystem = coordsys
    #     clip_plane.projectorMap = {}
    #     return clip_plane

    # def create_clip_rect(self, name, x, y, z, w, h, csys, active=True):
    #     clip_rect = thrift_interface.ClipRect()
    #     clip_rect.name = name
    #     clip_rect.position = self.create_3d_point(x, y, z)
    #     clip_rect.activated = active
    #     clip_rect.size = self.create_2d_point(w, h)
    #     clip_rect.showOutline = False
    #     clip_rect.coordinateSystem = csys
    #     clip_rect.clippingActivated = True
    #     return clip_rect
