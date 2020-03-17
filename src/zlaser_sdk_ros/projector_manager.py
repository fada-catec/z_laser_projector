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
        self.reference_object_list = []


        # self.projection_group = "my_group"
        # self.do_register_coordinate_system = False
        # self.do_target_search = False
        self.geo_tree_elements = []

        
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
        if hasattr(self,'reference_object_name'):
            self.thrift_client.RemoveGeoTreeElem(self.reference_object_name) # necesario? se pone mejor en un método aparte remove_geo_tree_elem no?
        self.clear_geo_tree() # al hacer deactivate en el proyector se borran automaticamente los geo_trees? es mejor dejar ese if para borrarlos? no hace falta?
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
            # sys.exit(1)
        except zlp.thrift_interface.FunctionModulePropertyBranchAlreadyInUse as e:
            return ("FunctionModulePropertyBranchAlreadyInUse: " + e.branchName)
            # sys.exit(1)
        except zlp.thrift_interface.FunctionModuleClassNotLicensed as e:
            return ("FunctionModuleClassNotLicensed: " + e.which)
            # sys.exit(1)

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
        return available_coordinate_systems

    def set_coordinate_system(self,coord_sys): # set_coord_sys = setting the object.coordinate_system property value
        self.coordinate_system = [coord_sys]
        return ("Setting [{}] as coordinate system".format(coord_sys))

    def define_coordinate_system(self,req):
        
        print("Create reference object: {}".format(req.name_ref_object))
        reference_object = zlp.create_reference_object()
        reference_object.name = req.name_ref_object.data
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

        self.thrift_client.SetReferenceobject(reference_object) # activate reference point to use for transformation
        # self.thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", reference_object.name)
        
        self.add_ref_object(reference_object)

        print("Reference object created. Coordinate system defined but not registered.")
        return (reference_object.name,reference_object.coordinateSystem)

    def __define_reference_point(self,reference_object,crossSize,n,d,x,y):
        reference_object.refPointList[n].tracePoint.x = x
        reference_object.refPointList[n].tracePoint.y = y
        reference_object.refPointList[n].distance = d
        reference_object.refPointList[n].activated = True
        reference_object.refPointList[n].crossSize = crossSize
        return reference_object

    def add_ref_object(self,ref_obj): 
        self.reference_object_list.append(ref_obj)
        print("[{}] appended".format(self.reference_object_list[-1]))
        print("Reference object list: [{}]".format(self.reference_object_list))

    def register_coordinate_system(self, ref_obj, cs):
        
        print("Registering coordinate system {}".format(cs))

        self.thrift_client.FunctionModuleSetProperty(self.module_id, "referenceData", ref_obj) #ref_obj = reference_object.name
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

    def show_coordinate_system(self,cs,secs):
        print("Projecting [{}] coordinate system for {} seconds".format(cs,secs))
        self.thrift_client.FunctionModuleSetProperty(self.module_id,"showAllRefPts","1")
        time.sleep(secs)
        self.thrift_client.FunctionModuleSetProperty(self.module_id,"showAllRefPts","0")



    def clear_geo_tree(self):
        for name in self.geo_tree_elements:
            self.thrift_client.RemoveGeoTreeElem(name)
        self.geo_tree_elements.clear()



    def start_projection(self):
        self.thrift_client.TriggerProjection(self.projector_id)

    def stop_projection(self):
        self.thrift_client.deactivate_projector(self.projector_id)


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

    