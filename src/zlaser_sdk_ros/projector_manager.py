#!/usr/bin/env python3

import sys
import os
import threading
import time
#from zlaser_sdk_ros import zlp
import zlp
import math

class ProjectorManager:
    def __init__(self):

        # Define default values
        self.projector_IP = "192.168.10.10"
        self.server_IP = "192.168.10.11"
        self.connection_port = 9090
        self.license_path = "Pendrive_ZLaser/1900027652.lic" #???
        
        # Auxiliar variables
        self.reference_object_list = []
        self.coordinate_system = ""
        # self.geo_tree_elements = []

        
        self.thrift_client = zlp.ThriftClient() # Create client object

    def client_server_connect(self):
        try:
            self.thrift_client.connect(self.server_IP, self.connection_port)
            return "Connected to server. You can activate projector now"
        except Exception as e:
            return e

    def client_server_disconnect(self):
        try:
            self.thrift_client.disconnect()
            return "Disconnected to server. End of connection"
        except Exception as e:
            return e

    def activate(self):
        try:
            projectors = self.thrift_client.scan_projectors(self.projector_IP)
            self.projector_id = projectors[0]
            print("Projector ID: ",self.projector_id) # cómo mostrar por pantalla ????
            self.thrift_client.activate_projector(self.projector_id)
            return "Projector activated. You can start the projection now"
        except Exception as e:
            return e

    def deactivate(self):
        # QUE HACER ADEMAS AL DEACTIVATE: ????
        # self.geo_tree_elements.clear() ?? <- geo_tree_elements[] es un es una lista de geo_tree_elem creada por nosotros (.clear() elimina los elementos del vector)
        # thrift_client.RemoveGeoTreeElem("") <- remove all??
        # thrift_client.FunctionModuleRelease(module_id) ??
        # if hasattr(self,'reference_object_name'):
            # self.thrift_client.RemoveGeoTreeElem(self.reference_object_name) # necesario? se pone mejor en un método aparte remove_geo_tree_elem no?
        # self.clear_geo_tree() # al hacer deactivate en el proyector se borran automaticamente los geo_trees? es mejor dejar ese if para borrarlos? no hace falta?
                                # se borran al desenchufar el proyector
        try:
            self.thrift_client.deactivate_projector(self.projector_id)
            return "Projector deactivated."
        except Exception as e:
            return e

    def transfer_license(self):
        try:
            license_path = os.path.abspath(self.license_path)
            license_file = os.path.basename(license_path)
            self.thrift_client.transfer_file(license_path, license_file, True)
        except zlp.thrift_interface.CantWriteFile as e:
            return e
        except FileNotFoundError:
            return "File not found!"
        self.thrift_client.LoadLicense(license_file)
        return "License transfered"

    def check_license(self):
        return self.thrift_client.CheckLicense()

    def function_module_create(self):
        self.module_id = ""
        try:
            self.module_id = self.thrift_client.FunctionModuleCreate("zFunctModRegister3d", "3DReg")
            return "Function Module Created"
        except zlp.thrift_interface.FunctionModuleClassNotRegistered as e:
            return ("FunctionModuleClassNotRegistered: " + e.which)
        except zlp.thrift_interface.FunctionModulePropertyBranchAlreadyInUse as e:
            return ("FunctionModulePropertyBranchAlreadyInUse: " + e.branchName)
        except zlp.thrift_interface.FunctionModuleClassNotLicensed as e:
            return ("FunctionModuleClassNotLicensed: " + e.which)

        # self.cv = threading.Condition()
        # self.thrift_client.set_function_module_state_changed_callback(self.function_module_changed_callback())

    # def function_module_changed_callback(self, old_state, new_state):
    #     if new_state != zlp.thrift_interface.FunctionModuleStates.RUNNING:
    #         self.cv.acquire()
    #         print("Function module stopped running.")
    #         print("Module", self.module_id, ":", old_state, "->", new_state)
    #         self.cv.notify()
    #         self.cv.release()

    def get_coordinate_systems(self):
        available_coordinate_systems = self.thrift_client.GetCoordinatesystemList()
        print("CURRENT COORDINATE SYSTEM: [{}]".format(self.coordinate_system))
        # allGeoTree = self.thrift_client.GetGeoTreeIds()
        # print("Available GeoTree:", allGeoTree)
        return available_coordinate_systems

    def define_coordinate_system(self,req):
        reference_object = zlp.create_reference_object()
        reference_object.name = "RefObj_" + req.name_cs.data
        print("Creating reference object: {}".format(reference_object.name))
        reference_object.coordinateSystem = req.name_cs.data
        reference_object.projectorID = self.projector_id

        reference_object.refPointList = [   zlp.create_reference_point("T1", req.T1_x.data, req.T1_y.data),
                                            zlp.create_reference_point("T2", req.T2_x.data, req.T2_y.data),
                                            zlp.create_reference_point("T3", req.T3_x.data, req.T3_y.data),
                                            zlp.create_reference_point("T4", req.T4_x.data, req.T4_y.data)]
        
        crossSize = zlp.create_2d_point(req.crossize_x.data,req.crossize_y.data) # set global crosssize for all reference points

        reference_object = self.__define_reference_point(reference_object,crossSize,0,req.distance.data,req.x1.data,req.y1.data) # define coordinates in user system [mm]
        reference_object = self.__define_reference_point(reference_object,crossSize,1,req.distance.data,req.x2.data,req.y2.data)
        reference_object = self.__define_reference_point(reference_object,crossSize,2,req.distance.data,req.x3.data,req.y3.data)
        reference_object = self.__define_reference_point(reference_object,crossSize,3,req.distance.data,req.x4.data,req.y4.data)

        # reference_object.activated = True # en set_cs
        # self.thrift_client.SetReferenceobject(reference_object) # en set_cs
        # self.thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", reference_object.name) # en set_cs
        
        self.add_ref_object(reference_object) 

        print("Reference object created. Coordinate system defined but not registered.")
        return (reference_object.coordinateSystem)

    def __define_reference_point(self,reference_object,crossSize,n,d,x,y):
        reference_object.refPointList[n].tracePoint.x = x
        reference_object.refPointList[n].tracePoint.y = y
        reference_object.refPointList[n].distance = d
        reference_object.refPointList[n].activated = True
        reference_object.refPointList[n].crossSize = crossSize
        return reference_object

    def add_ref_object(self,ref_obj): 
        self.reference_object_list.append(ref_obj)
        # print(self.reference_object_list[:])
        print("[{}] appended".format(self.reference_object_list[-1].name))
        # print("Reference object list: [{}]".format(self.reference_object_list[:].name))
        # print("Reference object list: [{}]".format(self.reference_object_list))
        print(type(self.reference_object_list))

    def set_coordinate_system(self,coord_sys): 
        # SI SE HACE DISCONNECT EN EL PROYECTOR SE PIERDE LA INFO DE LOS REF_OBJECTS -> hay que eliminarlos todos en el disconnect
        # porque se borra la info de los ref_obj pero no los nombres de los coord sys??
        
        ref_obj_name = "RefObj_" + coord_sys 

        for i in range (0,len(self.reference_object_list)):
            if self.reference_object_list[i].name == ref_obj_name:
                self.reference_object_list[i].activated = True
                self.thrift_client.SetReferenceobject(self.reference_object_list[i])
                self.thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", self.reference_object_list[i].name)
            else:
                self.reference_object_list[i].activated = False
                self.thrift_client.SetReferenceobject(self.reference_object_list[i])
        
        self.coordinate_system = coord_sys # set the object.coordinate_system property value to use it wherever - HACE FALTA???

        # allReferenceObjects = self.thrift_client.GetGeoTreeIds()
        # print("Available reference objects:", allReferenceObjects)

        # ref_obj = self.thrift_client.GetReferenceobject("RefObject1")
        # print(ref_obj)

        return ("Setting [{}] as coordinate system".format(coord_sys))

    def register_coordinate_system(self,coord_sys):
        print("Registering coordinate system {}".format(coord_sys))

        self.thrift_client.FunctionModuleSetProperty(self.module_id, "runMode", "1")
        
        # self.cv.acquire()
        self.thrift_client.FunctionModuleRun(self.module_id) # Calculate transformation
        # self.cv.wait()
        # self.cv.release()

        state = self.thrift_client.FunctionModuleGetProperty(self.module_id, "state")
        if state != "1":  # idle
            return "Function module is not in idle state, hence an error has occured."
        else:
            return "Finished to register coordinate system on projector"

    def show_coordinate_system(self,coord_sys,secs):
        print("Projecting [{}] coordinate system for {} seconds".format(coord_sys,secs))
        self.thrift_client.FunctionModuleSetProperty(self.module_id,"showAllRefPts","1")
        time.sleep(secs)
        self.thrift_client.FunctionModuleSetProperty(self.module_id,"showAllRefPts","0")
        return "Finished to show coordinate system"

    def remove_coordinate_system(self,coord_sys):
        # for name in self.geo_tree_elements:
            # self.thrift_client.RemoveGeoTreeElem(name)
        #self.geo_tree_elements.clear()

        reference_object_name = "RefObj_" + coord_sys
        self.thrift_client.RemoveGeoTreeElem(reference_object_name)
        return("Coordinate system [{}] removed".format(coord_sys))


    # def set_up_projector(self):
    #     self.clientServerConnect()
    #     self.activate()
    #     self.transferLicense()
    #     name = self.projection_group + "/my_circle" + id
    #     self.geo_tree_elements.append(name)
    #     circle = zlp.create_circle(x, y, r, name)
    #     circle.activated = True
    #     circle.coordinateSystemList = self.coordinate_system
    #     try:
    #         self.thrift_client.SetCircleSegment(circle)
    #         return "Defined a circle segment to project"
    #     except Exception as e:
    #         return e

    def start_projection(self):
        self.thrift_client.TriggerProjection()
        return(" ----- PROJECTING ----- ")

    def stop_projection(self):
        self.thrift_client.deactivate_projector(self.projector_id)
        return(" ----- STOP PROJECTION ----- ")

    def create_polyline(self,projection_group,id,x,y,angle,r):
        polyline_name = projection_group + "/my_polyline_" + id
        polyline = zlp.create_polyline(polyline_name)
        # self.geo_tree_elements.append(name)

        # linestring = [ zlp.create_3d_point(x, y),
                    #    zlp.create_3d_point(x+r*math.cos(angle*math.pi/180), y+r*math.sin(angle*math.pi/180))]

        linestring = [ zlp.create_3d_point(x, y),
                       zlp.create_3d_point(x+100, y+100)]
        
        polyline.polylineList = [linestring]
        polyline.activated = True
        polyline.coordinateSystemList = [self.coordinate_system]
        try:
            self.thrift_client.SetPolyLine(polyline)
            print("Projecting shape for 5 seconds in order to check the shape")
            # self.start_projection()
            return "Defined a cross segment to project"
        except Exception as e:
            return e

    def create_circle(self,projection_group,id,x,y,r):
        circle_name = projection_group + "/my_circle_" + id
        circle = zlp.create_circle(x,y,r,circle_name)

        circle.activated = True
        circle.coordinateSystemList = self.coordinate_system
        try:
            self.thrift_client.SetCircleSegment(circle)
            print("Projecting shape for 5 seconds in order to check the shape")
            # self.start_projection()
            self.thrift_client.TriggerProjection()
            return "Defined a circle segment to project"
        except Exception as e:
            return e

    def hide_shape(self,projection_group,shape_name,id): # deactivate shape

        if shape_name == "polyline":
            polyline = self.thrift_client.GetPolyLine(projection_group + "/my_" + shape_name + "_" + id)
            polyline.activated = False
            self.thrift_client.SetPolyLine(polyline)

        # reference_object.activated = False  # <- puede servir 

        # name = projection_group + "/my_" + shape_name + "_" + id
        # shape = self.thrift_client.GetProjectionElement(name)
        # shape.activated = False
        # if shape_name == "polyline":
        #     self.thrift_client.SetPolyLine(polyline)
        
        self.start_projection()

    def remove_shape(self,projection_group,shape_name,id):
        self.thrift_client.RemoveGeoTreeElem(projection_group + "/my_" + shape_name + "_" + id)
        # self.start_projection()

    def remove_group(self,projection_group):
        self.thrift_client.RemoveGeoTreeElem(projection_group)
        # self.start_projection()
