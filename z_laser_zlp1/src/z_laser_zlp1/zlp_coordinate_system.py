#!/usr/bin/env python3

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
This module contains utility classes and methods which ease the usage of the thrift interface to ZLP Service."""

import os
import sys
import threading 

from z_laser_zlp1.zlp_utils import UserT, GeometryTool

class CoordinateSystem(object):
    """This class implement the functions related with coordinate systems management.
    
    Args:
        projector_id (str): serial number of the projector
        module_id (str): function module identification name
        thrift_client (object): object with the generated client to communicate with the projector

    Attributes:
        projector_id (str): serial number of the projector
        module_id (str): function module identification name
        reference_object_list (list): list of created reference objects
    """
    def __init__(self, projector_id, module_id, thrift_client):
        """Initialize the CoordinateSystem object."""
        self.__thrift_client = thrift_client 
        self.__geometry_tool = GeometryTool(thrift_client)

        self.projector_id = projector_id
        self.module_id = module_id
        self.reference_object_list = []

        self.cv = threading.Condition()

    def coordinate_system_list(self):
        """Get list of active available coordinate systems from projector.

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
        ref_obj = self.__thrift_client.thrift_interface.Referenceobject()
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
        ref_point = self.__thrift_client.thrift_interface.Referencepoint()
        ref_point.name = name
        ref_point.refPoint = self.__geometry_tool.create_3d_point(x, y, z)
        ref_point.activated = active
        ref_point.tracePoint = self.__geometry_tool.create_2d_point()
        ref_point.crossSize = self.__geometry_tool.create_2d_point()
        ref_point.distance = 0
        return ref_point

    def define_cs(self, cs, do_target_search):
        """Generate new coordinate system.

        Args:
            cs (object): struct with the necessary parameters to create the coordinate system 
            do_target_search(bool): 

        Returns:
            tuple[str, bool, str]: the first value in the returned tuple is the name string of the coordinate system generated, 
            the second is a bool success value and the third value in the tuple is an information message string
        """
        try:
            reference_object = self.create_reference_object()
            reference_object.name = cs.name
            reference_object.coordinateSystem = cs.name
            reference_object.projectorID = self.projector_id

            resolution = cs.resolution
            d = cs.distance
            cross_size = self.__geometry_tool.create_2d_point(d*0.02, d*0.02) # 20% of distance

            size_horiz = cs.P[1].x - cs.P[0].x
            size_vert  = cs.P[3].y - cs.P[0].y

            T = UserT(cs.T[0].x, cs.T[0].y, resolution, size_horiz, size_vert).T

            for i in range(4):
                reference_object.refPointList.append(self.create_reference_point("T"+str(i), T[i].x, T[i].y))

            for i in range(4):
                reference_object = self.__define_reference_point(reference_object, cross_size, i, d, cs.P[i].x, cs.P[i].y)

            self.__ref_obj_state(False, reference_object)
            self.__thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", reference_object.name)

            if do_target_search:
                success,message,result = self.scan_targets(reference_object.name)
                if success:
                    cs = self.update_cs(cs, result)
                    message = "Cordinate system defined and scanned correctly."
            else:
                success = True
                message = "Cordinate system defined correctly"
                cs = []

        except Exception as e:
            self.__thrift_client.RemoveGeoTreeElem(cs.name)
            success = False 
            message = e
            cs = []

        return success,message,cs

    def update_cs(self, cs, scan_result):
        cs.P[0].x = float(scan_result[0])
        cs.P[0].y = float(scan_result[1])
        cs.P[1].x = float(scan_result[2])
        cs.P[1].y = float(scan_result[3])
        cs.P[2].x = float(scan_result[4])
        cs.P[2].y = float(scan_result[5])
        cs.P[3].x = float(scan_result[6])
        cs.P[3].y = float(scan_result[7])
        return cs

    def function_module_changed_callback(self, module_id, old_state, new_state):
        """Callback for function module state change, events handler. It is used for stopping running code while the projector is 
        scanning targets.

        Args:
            module_id (str): function module identification name
            old_state (str): old state of the function module
            new_state (str): new state of the function module
        """
        if new_state != self.__thrift_client.thrift_interface.FunctionModuleStates.RUNNING:
            self.cv.acquire()
            self.cv.notify()
            self.cv.release()

    def scan_targets(self, ref_obj_name):
        """Projector automatic scanning of the targets placed on the projection surface.

        Args:
            ref_obj_name (str): object with the necessary parameters to create the coordinate system 

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is an 
            information message string
        """
        result_tracepoints = []
        try:
            self.__thrift_client.FunctionModuleSetProperty(self.module_id, "runMode", "0")
            point_search_prop_path = "config.projectorManager.projectors." + self.projector_id + ".search"
            self.__thrift_client.SetProperty(point_search_prop_path + ".threshold", "1")
            self.__thrift_client.set_function_module_state_changed_callback(self.function_module_changed_callback)

            self.cv.acquire()
            self.__thrift_client.FunctionModuleRun(self.module_id)
            self.cv.wait()
            self.cv.release()

            state = self.__thrift_client.FunctionModuleGetProperty(self.module_id, "state")
            if state != "1": 
                success = False
                message = "Function module is not in idle state, hence an error has occured."
            else:
                reference_object = self.__thrift_client.GetReferenceobject(ref_obj_name)
                found_all = True
                for i in range(0, len(reference_object.refPointList)):
                    resultPath = "result.tracePoints." + str(i)
                    if self.__thrift_client.FunctionModuleGetProperty(self.module_id, resultPath + ".found") != "true":
                        found_all = False
                        break
            
                if not found_all:
                    success = False
                    message = "Not all targets have been found."
                    
                else:
                    success = True
                    message = "Targets scanned successfully."

                    for i in range(4):
                        property_x = "result.tracePoints.%d.x" % i
                        property_y = "result.tracePoints.%d.y" % i
                        result_tracepoints.append(self.__thrift_client.FunctionModuleGetProperty(self.module_id, property_x))
                        result_tracepoints.append(self.__thrift_client.FunctionModuleGetProperty(self.module_id, property_y))

        except Exception as e:
            success = False 
            message = e
        
        return success,message,result_tracepoints

    def __define_reference_point(self, reference_object, cross_size, n, d, x, y):
        """Fill other fields of the coordinate system structure.

        Args:
            reference_object (object): object of the active coordinate system
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

    def register_cs(self, coord_sys):
        """Register the new coordinate system at the projector once it has been generated and activated.

        Args:
            coord_sys (str): name of the coordinate system

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            self.__thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", coord_sys)
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

    def set_cs(self, coord_sys): 
        """Activate the new generated coordinate system and deactivate the other existing coordinate systems at the projector.

        Args:
            coord_sys (str): name of the new coordinate system
        
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            geo_tree_list = self.__thrift_client.GetGeoTreeIds()
            matches = [geo_tree_id.name for geo_tree_id in geo_tree_list if geo_tree_id.elemType == 4096]
            coordinate_systems = [self.__thrift_client.GetReferenceobject(name) for name in matches]
            
            if any(coord_sys in cs.name for cs in coordinate_systems):
                for cs in coordinate_systems:
                    if cs.name == coord_sys:
                        self.__ref_obj_state(True, cs)
                        self.__thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", cs.name)
                    else:
                        self.__ref_obj_state(False, cs)
                
                success = True
                message = coord_sys + " set as active coordinate system"
            else:
                success = False
                message = coord_sys + " does not exist"

        except Exception as e:
            success = False 
            message = e

        return success,message

    def show_cs(self, coord_sys):
        """Project a coordinate system on the projection surface.

        Args:
            coord_sys (str): name of the coordinate system
            secs (int): number of seconds the projection lasts

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            self.__thrift_client.FunctionModuleSetProperty(self.module_id,"showAllRefPts","1")
            
            success = True
            message = "Show [" + coord_sys + "] coordinate system"
        
        except Exception as e:
            success = False 
            message = e

        return success,message

    def hide_cs(self, coord_sys):
        """Project a coordinate system on the projection surface.

        Args:
            coord_sys (str): name of the coordinate system
            secs (int): number of seconds the projection lasts

        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            self.__thrift_client.FunctionModuleSetProperty(self.module_id,"showAllRefPts","0")
            
            success = True
            message = ("Hide [{}] coordinate system".format(coord_sys))
        
        except Exception as e:
            success = False 
            message = e

        return success,message

    def remove_cs(self, coord_sys):
        """Delete a coordinate system.
            
        Args:
            coord_sys (str): name of the coordinate system
                
        Returns:
            tuple[bool, str]: the first value in the returned tuple is a bool success value and the second value in the tuple is 
            an information message string
        """
        try:
            self.__thrift_client.RemoveGeoTreeElem(coord_sys)
            success = True
            message = "Coordinate system [" + coord_sys + "] removed. Set other coordinate system or define a new one before continue."

        except Exception as e:
            success = False 
            message = e

        return success,message

    def get_cs(self, coord_sys, cs_params):
        """Get the parameters values of a defined coordinate system.
            
        Args:
            coord_sys (str): name of the coordinate system
                
        Returns:
            tuple[object, bool, str]: the first value in the returned tuple is an object with the coordinate system parameters values,
            the second is a bool success value and the third value in the tuple is an information message string
        """
        try:
            cs_ref_obj         = self.__thrift_client.GetReferenceobject(coord_sys)

            cs_params.name     = cs_ref_obj.coordinateSystem
            cs_params.distance = cs_ref_obj.refPointList[0].distance
            for i in range(4):
                cs_params.P[i].x = cs_ref_obj.refPointList[i].tracePoint.x
                cs_params.P[i].y = cs_ref_obj.refPointList[i].tracePoint.y
                cs_params.T[i].x = cs_ref_obj.refPointList[i].refPoint.x
                cs_params.T[i].y = cs_ref_obj.refPointList[i].refPoint.y
                cs_params.T[i].z = cs_params.distance
            
            success = True
            message = "[" + coord_sys + "] coordinate system params returned."

        except Exception as e:
            success = False 
            message = e
            cs_params = []

        return cs_params,success,message