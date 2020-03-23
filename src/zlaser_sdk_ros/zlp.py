#! python
# -*- coding: utf-8 -*-
"""Helper module for python thrift interface to ZLP Service.

This module contains utility classes and methods which ease the usage of the thrift interface to ZLP Service.
"""

import os
import time
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

    def connect(self, ip, port):
        """Connects the client to ZLP-Service and establishes an event channel if needed.

        Args:
            ip: ipv6 network address of ZLP-Service
            port: port number on which ZLP-Service listens for requests
        """
        self.__init_client(ip, port)
        self.__init_event_channel()

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

    def disconnect(self):
        """Disconnect from ZLP Service thrift server and close own event server."""
        self.DisconnectClientEventChannel()
        self.close()

        if self._event_channel:
            self._event_channel.close()

    def transfer_file(self, local_path, remote_file, overwrite=False):
        content = open(local_path, 'r').read()
        self.TransferDataToFile(content, remote_file, overwrite)

    def get_projectors(self, scan=False, scan_addresses=""):
        """Gets a list of active projectors or scan the network for projectors.

        Args:
            scan: scan the network for projectors
            addresses: addresses or address are to scan

        Returns:
            list: serial numbers of the found projectors
        """
        try:
            # set parameters and start the search
            if scan:
                self.SetProperty("config.projectorManager.cmdGetProjectors.scan", "1")
            else:
                self.SetProperty("config.projectorManager.cmdGetProjectors.scan", "0")

            self.SetProperty("config.projectorManager.cmdGetProjectors.scanAddresses", scan_addresses)
            self.SetProperty("config.projectorManager.cmdGetProjectors", "1")

            # Get the results
            serial_list = self.GetProperty("config.projectorManager.cmdGetProjectors.result.entries")

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
            list: serial numbers of the found projectors
        """
        log.info("Scanning for projectors...")
        serial_list = self.get_projectors(True, addresses)
        if (len(serial_list) == 0):
            log.warning("No projectors were found!")
            return []

        log.info("Available projectors: " + str(serial_list))
        return serial_list

    def activate_projector(self, projector_serial):
        """Activate a projector.

        Args:
            projector_serial: serial number of the projector
        """
        log.info("Activating projector: " + projector_serial)

        try:
            self.SetProperty("config.projectorManager.cmdActivateProjector.serial", projector_serial)
            self.SetProperty("config.projectorManager.cmdActivateProjector.active", "1")
            self.SetProperty("config.projectorManager.cmdActivateProjector", "1")
            #blocks until the projector is activated
            self.get_projectors()
        except Exception as e:
            log.error("Could not activate projector:", projector_serial)
            raise



    def deactivate_projector(self, projector_serial):
        """Deactivates a projector.

        Args:
            projector_serial: serial number of the projector
        """

        try:
            log.info("Deactivating projection of projector: " + projector_serial)
            projector_property_path = "config.projectorManager.projectors." + projector_serial
            self.SetProperty(projector_property_path + ".cmdShowProjection.show", "0")
            self.SetProperty(projector_property_path + ".cmdShowProjection", "1")

            log.info("Deactivating projector: " + projector_serial)
            self.SetProperty("config.projectorManager.cmdActivateProjector.serial", projector_serial)
            self.SetProperty("config.projectorManager.cmdActivateProjector.active", "0")
            self.SetProperty("config.projectorManager.cmdActivateProjector", "1")

        except Exception as e:
            log.error("Error: projector could not be deactivated")
            raise e


def create_matrix4x4():
    mat = thrift_interface.Matrix4x4(list())
    return mat

# use the default elements to set default values for all new projection elements
default_projection_element = thrift_interface.ProjectionElement()
default_projection_element.pen = 0
default_projection_element.coordinateSystemList = []
default_projection_element.projectorIDList = []
default_projection_element.userTrans = create_matrix4x4()
default_projection_element.activated = True


def init_element_3d(elem):
    elem.userTrans = create_matrix4x4()
    return elem

def init_projection_element(elem):
    elem.coordinateSystemList = copy.deepcopy(default_projection_element.coordinateSystemList)
    elem.projectorIDList = copy.deepcopy(default_projection_element.projectorIDList)
    elem.userTrans = copy.deepcopy(default_projection_element.userTrans)
    elem.activated = default_projection_element.activated
    elem.pen = default_projection_element.pen
    # `elem.detectReflection` has a default value
    return elem


def create_projection_element(name):
    projection_element = copy.deepcopy(default_projection_element)
    projection_element.name = name
    return projection_element


def create_polyline(name):
    polyline = thrift_interface.Polyline()
    polyline = init_projection_element(polyline)
    polyline.name = name
    polyline.polylineList = []
    return polyline


def create_circle(x, y, r, name):
    circle = thrift_interface.CircleSegment()
    circle = init_projection_element(circle)
    circle.name = name
    circle.radius = r
    circle.center = create_3d_point(x, y)
    return circle


def create_oval(x, y, w, h, name, angle=0):
    oval = thrift_interface.OvalSegment()
    oval = init_projection_element(oval)
    oval.name = name
    oval.width = w
    oval.height = h
    oval.angle = angle
    oval.center = create_3d_point(x, y)
    return oval

def create_text_element(x, y, text, name, height):
    textelem = thrift_interface.TextElement()
    textelem = init_projection_element(textelem)
    textelem.name = name
    textelem.position = create_3d_point(x, y)
    textelem.text = text
    textelem.height = height
    return textelem

def create_2d_point(x=0, y=0):
    return thrift_interface.Vector2D(x, y)


def create_3d_point(x=0, y=0, z=0):
    return thrift_interface.Vector3D(x, y, z)

# def create_2d_point(x=0, y=0):
#     return thrift_interface.Vector3D(x, y)


def create_reference_object():
    ref_obj = thrift_interface.Referenceobject()
    ref_obj.name = ""
    ref_obj.activated = False
    ref_obj.fieldTransMat = create_matrix4x4()
    ref_obj.refPointList = []
    ref_obj.projectorID = ""
    ref_obj.coordinateSystem = ""
    return ref_obj

def create_reference_point(name, x, y, z=0, active=True):
    ref_point = thrift_interface.Referencepoint()
    ref_point.name = name
    ref_point.refPoint = create_3d_point(x, y, z)
    ref_point.activated = active
    ref_point.tracePoint = create_2d_point()
    ref_point.crossSize = create_2d_point()
    ref_point.distance = 0
    return ref_point

def create_driftcompensation_object(name="", active=True):
    dc_obj = thrift_interface.DriftCompensationObject()
    dc_obj.name = name
    dc_obj.activated = active
    dc_obj.compensationTransMat = create_matrix4x4()
    dc_obj.dcPointList = []
    dc_obj.projectorID = ""
    dc_obj.fixedTranslation = [False,False,False]
    dc_obj.fixedRotation = [False,False,False]
    return dc_obj

def create_driftcompensation_point(name, x=0, y=0, z=0, distanceToOrigin=0, active=True):
    dc_point = thrift_interface.DriftCompensationPoint()
    dc_point.name = name
    dc_point.activated = active
    dc_point.isReserve = False
    dc_point.traceVector = create_3d_point(x, y, z)
    dc_point.crossSize = 100
    dc_point.distanceToOrigin = distanceToOrigin
    dc_point.weight = 1
    return dc_point

def create_clip_group(name, type=thrift_interface.ClipSetTypes.Union, active=True):
    clip_group = thrift_interface.ClipGroup()
    clip_group.name = name
    clip_group.projectorMap = {}
    clip_group.coordinateSystem = ""
    clip_group.setType = type
    clip_group.activated = active
    return clip_group

def create_clip_plane(name, x, y, z, normal, coordsys, active=True):
    clip_plane = thrift_interface.ClipPlane()
    clip_plane.name = name
    clip_plane.point = create_3d_point(x, y, z)
    clip_plane.normal = normal
    clip_plane.activated = active
    clip_plane.coordinateSystem = coordsys
    clip_plane.projectorMap = {}
    return clip_plane

def create_clip_rect(name, x, y, z, w, h, csys, active=True):
    clip_rect = thrift_interface.ClipRect()
    clip_rect.name = name
    clip_rect.position = create_3d_point(x, y, z)
    clip_rect.activated = active
    clip_rect.size = create_2d_point(w, h)
    clip_rect.showOutline = False
    clip_rect.coordinateSystem = csys
    clip_rect.clippingActivated = True
    return clip_rect
