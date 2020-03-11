#!/usr/bin/env python3

import sys
import os
import threading
import time
#from zlaser_sdk_ros import zlp
import zlp

class ProjectorManager:
    def __init__(self):

        # Define default values
        self.projector_IP = "192.168.10.10"
        self.server_IP = "192.168.10.11"
        self.connection_port = 9090
        self.license_path = "Pendrive_ZLaser/1900027652.lic" #???
        self.projection_group = "my_group"
        self.do_register_coordinate_system = False
        self.do_target_search = False
        self.geo_tree_elements = []

        # Create client object
        self.thrift_client = zlp.ThriftClient()


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
            print(self.projector_id) # mostrar por pantalla ????
            self.thrift_client.activate_projector(self.projector_id)
            return "Projector activated. You can start the projection now"
        except Exception as e:
            return e

    def deactivate(self):
        if hasattr(self,'reference_object_name'):
            self.thrift_client.RemoveGeoTreeElem(self.reference_object_name) # necesario?
        self.clear_geo_tree()
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

    def start_projection(self):
        self.thrift_client.TriggerProjection(self.projector_id)

    def stop_projection(self):
        self.thrift_client.deactivate_projector(self.projector_id)

    def clear_geo_tree(self):
        for name in self.geo_tree_elements:
            self.thrift_client.RemoveGeoTreeElem(name)
        self.geo_tree_elements.clear()

    def get_coordinate_systems(self):
        available_coordinate_systems = self.thrift_client.GetCoordinatesystemList()
        return available_coordinate_systems

    def show_coordinate_system(self,secs):
        module_id = self.thrift_client.FunctionModuleCreate("zFunctModRegister3d", "3DReg")
        self.thrift_client.FunctionModuleSetProperty(module_id,"showAllRefPts","1")
        time.sleep(secs)
        self.thrift_client.deactivate_projector(self.projector_id)

    def set_coordinate_system(self,coord_sys):
        self.coordinate_system = [coord_sys]
        return self.coordinate_system

    def __define_reference_point(self,crossSize,n,d,x,y):
        self.reference_object.refPointList[n].tracePoint.x = x
        self.reference_object.refPointList[n].tracePoint.y = y
        self.reference_object.refPointList[n].distance = d
        self.reference_object.refPointList[n].activated = True
        self.reference_object.refPointList[n].crossSize = crossSize

    def define_coordinate_system(self,cs_name):
        self.reference_object = zlp.create_reference_object()
        self.reference_object_name = "RefObject"
        #print("Create the reference object...")
        self.reference_object.name = self.reference_object_name
        self.reference_object.refPointList = [  zlp.create_reference_point("T1", 0, 0),
                                                zlp.create_reference_point("T2", 1500, 0),
                                                zlp.create_reference_point("T3", 0, 1500),
                                                zlp.create_reference_point("T4", 1500, 1500)  ]
        # set global crosssize for all reference points
        crossSize = zlp.create_2d_point(50,50)
        # - define coordinates in system of factory calibration wall [mm]
        # - activate reference point to use for transformation
        # - set cross size to set search area
        self.__defineReferencePoint(crossSize,0,3533.4,-1000,0)
        self.__defineReferencePoint(crossSize,1,3533.4,1000,0)
        self.__defineReferencePoint(crossSize,2,3533.4,-1000,2000)
        self.__defineReferencePoint(crossSize,3,3533.4,1000,2000)

        self.reference_object.coordinateSystem = cs_name
        self.reference_object.projectorID = self.projector_id
        self.coordinate_system = self.reference_object
        self.thrift_client.SetReferenceobject(self.reference_object)
        res = "Created reference object. Coordinate system is not registered."
        if self.do_register_coordinate_system: 
            res = self.registerCoordinateSystem()
        return res

    def register_coordinate_system(self):
        module_id = ""
        try:
            module_id = self.thrift_client.FunctionModuleCreate("zFunctModRegister3d", "3DReg")
        except zlp.thrift_interface.FunctionModuleClassNotRegistered as e:
            print("FunctionModuleClassNotRegistered: " + e.which)
            # sys.exit(1)
        except zlp.thrift_interface.FunctionModulePropertyBranchAlreadyInUse as e:
            print("FunctionModulePropertyBranchAlreadyInUse: " + e.branchName)
            # sys.exit(1)
        except zlp.thrift_interface.FunctionModuleClassNotLicensed as e:
            print("FunctionModuleClassNotLicensed: " + e.which)
            # sys.exit(1)
        self.thrift_client.FunctionModuleSetProperty(module_id, "referenceData", self.coordinate_system.name)
        cv = threading.Condition()
        def function_module_changed_callback(module_id, old_state, new_state):
            if new_state != zlp.thrift_interface.FunctionModuleStates.RUNNING:
                cv.acquire()
                #print("Function module stopped running.")
                # print("Module", module_id, ":", old_state, "->", new_state)
                cv.notify()
                cv.release()
        self.thrift_client.set_function_module_state_changed_callback(function_module_changed_callback)
        if self.do_target_search: 
            self.searchTargets()
        else: 
            # register projector to coordinate system
            self.thrift_client.FunctionModuleSetProperty(module_id, "runMode", "1")
            #print("Register projector to coordinate system...")
            cv.acquire()
            #print("Calculate transformation...")
            self.thrift_client.FunctionModuleRun(module_id)
            time.sleep(2)
            # cv.wait()
            # cv.release()
            state = self.thrift_client.FunctionModuleGetProperty(module_id, "state")
            if state != "1":  # idle
                return "Function module is not in idle state, hence an error has occured."
            else:
                res = self.thrift_client.FunctionModuleGetProperty(module_id, "result.averageDistance")
                #Activate reference object only if the calculated transformation was succesfully
                self.reference_object = self.thrift_client.GetReferenceobject(self.reference_object_name)
                self.reference_object.activated = True
                self.thrift_client.SetReferenceobject(self.reference_object)
                # print("Finished to register projector (aveDist:",res,")\n")
                # self.thrift_client.RemoveGeoTreeElem(self.reference_object_name)
                # self.thrift_client.FunctionModuleRelease(module_id)
                # self.thrift_client.deactivate_projector(self.projector_id)
                return "Finished to register coordinate system on projector"


        print("Do point search...")
        # set up reflector point search
        self.thrift_client.FunctionModuleSetProperty(module_id, "runMode", "0")
        point_search_prop_path = "config.projectorManager.projectors." + projector + ".search"
        self.thrift_client.SetProperty(point_search_prop_path + ".threshold", "5")
        # run point search
        cv.acquire()
        print("Reference point search is running...")
        self.thrift_client.FunctionModuleRun(module_id)
        cv.wait()
        cv.release()
        state = self.thrift_client.FunctionModuleGetProperty(module_id, "state")
        if state != "1":  # idle
            print("Function module is not in idle state, hence an error has occured.")
            sys.exit(1)
        print("Finished point search")
        # Check if all points were found
        self.reference_object = self.thrift_client.GetReferenceobject(self.reference_object_name)
        found_all = True
        for i in range(0, len(self.reference_object.refPointList)):
            resultPath = "result.tracePoints." + str(i)
            if self.thrift_client.FunctionModuleGetProperty(module_id, resultPath + ".found") != "true":
                found_all = False
                break
        if not found_all:
            print("Not all points have been found!")
        else:
            print("All points have been found.")

    def set_up_projector(self):
        self.clientServerConnect()
        self.activate()
        self.transferLicense()

        name = self.projection_group + "/my_circle" + id
        self.geo_tree_elements.append(name)
        circle = zlp.create_circle(x, y, r, name)
        circle.activated = True
        circle.coordinateSystemList = self.coordinate_system
        try:
            self.thrift_client.SetCircleSegment(circle)
            return "Defined a circle segment to project"
        except Exception as e:
            return e
    
    def create_polyline(self,id):
        name = self.projection_group + "/my_polyline" + id
        polyline = zlp.create_polyline(name)
        self.geo_tree_elements.append(name)
        linestringA = [ zlp.create_3d_point(-100, -100),
                        zlp.create_3d_point(+100, -100),
                        zlp.create_3d_point(+100, +100) ]
        linestringB = [ zlp.create_3d_point(-200, -200),
                        zlp.create_3d_point(+200, -200),
                        zlp.create_3d_point(+200, +200) ]
        polyline.polylineList = [linestringA, linestringB]
        polyline.activated = True
        polyline.coordinateSystemList = self.coordinate_system
        try:
            self.thrift_client.SetPolyLine(polyline)
            return "Defined a cross segment to project"
        except Exception as e:
            return e


        name = self.projection_group + "/my_text"
        text = zlp.create_text_element(x, y, text, name, 100)
        text.charSpacing = 100
        text.angle = 25
        text.activated = True
        text.coordinateSystemList = self.coordinate_system
        self.thrift_client.SetTextElement(text)

    def clean_projection(self):
        circle.activated   = False
        polyline.activated = False
        arc.activated      = False
        text.activated     = False
        self.thrift_client.SetCircleSegment(circle)
        self.thrift_client.SetPolyLine(polyline)
        self.thrift_client.SetCircleSegment(arc)
        self.thrift_client.SetTextElement(text)
        self.startProjection()                          # TODO revise this

    