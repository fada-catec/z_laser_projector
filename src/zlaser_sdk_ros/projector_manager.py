#!/usr/bin/env python3

# import sys
import os
import threading
from zlaser_sdk_ros import zlp

class ProjectorManager:
    def __init__(self):

        # Define default values
        self.projector_IP = "192.168.10.10"
        self.server_IP = "192.168.10.11"
        self.connection_port = 9090
        self.license_path = "/home"
        self.projection_group = "my_group"
        self.do_register_coordinate_system = False
        self.do_target_search = False

        # Create client object
        self.thrift_client = zlp.ThriftClient(event_handler=None)


    def clientServerConnect(self):
        try:
            self.thrift_client.connect(self.server_IP, self.connection_port)
            return "Connected to server. You can activate projector now"
        except Exception as e:
            return e

    def activateProjector(self):
        try:
            projectors = self.thrift_client.scan_projectors(self.projector_IP)
            self.projector_id = projectors[0]
            self.thrift_client.activate_projector(self.projector_id)
        except Exception as e:
            rospy.logerr("Exception: ",e)

    def deactivateProjector(self):
        try:
            self.thrift_client.deactivate_projector(self.projector_id)
            self.thrift_client.disconnect()
        except Exception as e:
            rospy.logerr("Exception: ",e)

    def transferLicense(self):
        try:
            print("Transferring license file...")
            license_path = os.path.abspath(self.license_path)
            license_file = os.path.basename(license_path)
            self.thrift_client.transfer_file(license_path, license_file, True)
        except zlp.thrift_interface.CantWriteFile as e:
            print(e.why, e.fileName)
        except FileNotFoundError:
            print("File not found!")

        self.thrift_client.LoadLicense(license_file);

        # set callback for property changed event TODO
        cv = threading.Condition()
        def property_changed_callback(prop, value):
            cv.acquire()
            print("Property changed: %s = %s" % (prop, value))
            cv.notify()
            cv.release()

        self.thrift_client.set_property_changed_callback(property_changed_callback)
        self.thrift_client.RegisterForChangedProperty("config.licenseState.IsValid")
        # activate projector and wait for callback
        cv.acquire()
        self.activateProjector()
        cv.wait()
        cv.release()
        # obtain license information
        valid = self.thrift_client.GetProperty("config.licenseState.IsValid")
        if valid == "true":
            print("The license is valid.")
            customer_name = self.thrift_client.GetProperty("license.Customer.Name")
            grantor_name = self.thrift_client.GetProperty("license.Grantor.Name")
            print("Customer: " + customer_name)
            print("Grantor: " + grantor_name)
            modules = self.thrift_client.GetPropChildren("config.licenseState.Modules")
            for m in modules:
                status = self.thrift_client.GetProperty("config.licenseState.Modules." + m)
                print("Function module: " + m + ", licensed: " + status)
        else:
            print("The license is invalid")

    def defineCoordinateSystem(self):
        reference_object = zlp.create_reference_object()
        reference_object_name = "RefObject"
        print("Create the reference object...")
        reference_object.name = reference_object_name
        reference_object.refPointList = [zlp.create_reference_point("T1", 0, 0),
                                        zlp.create_reference_point("T2", 1000, 0),
                                        zlp.create_reference_point("T3", 0, 1000),
                                        zlp.create_reference_point("T4", 1000, 1000)]
        # set global crosssize for all reference points
        crossSize = zlp.create_2d_point(50,50)
        # - define coordinates in system of factory calibration wall [mm]
        # - activate reference point to use for transformation
        # - set cross size to set search area
        reference_object.refPointList[0].tracePoint.x = 143.2
        reference_object.refPointList[0].tracePoint.y = -283.9
        reference_object.refPointList[0].distance = 3533.4
        reference_object.refPointList[0].activated = True
        reference_object.refPointList[0].crossSize = crossSize

        reference_object.refPointList[1].tracePoint.x = 151.1
        reference_object.refPointList[1].tracePoint.y = 264.8
        reference_object.refPointList[1].distance = 3533.4
        reference_object.refPointList[1].activated = True
        reference_object.refPointList[1].crossSize = crossSize

        reference_object.refPointList[2].tracePoint.x = -351.0
        reference_object.refPointList[2].tracePoint.y = -283.9
        reference_object.refPointList[2].distance = 3533.4
        reference_object.refPointList[2].activated = True
        reference_object.refPointList[2].crossSize = crossSize

        reference_object.refPointList[3].tracePoint.x = -339.7
        reference_object.refPointList[3].tracePoint.y = 278.3
        reference_object.refPointList[3].distance = 3533.4
        reference_object.refPointList[3].activated = True
        reference_object.refPointList[3].crossSize = crossSize

        reference_object.coordinateSystem = "DefinedCoordinateSystem"
        reference_object.projectorID = projector
        self.coordinate_system = reference_object
        self.thrift_client.SetReferenceobject(reference_object)
        if self.do_register_coordinate_system: 
            self.registerCoordinateSystem()

    def registerCoordinateSystem(self):
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
                print("Function module stopped running.")
                print("Module", module_id, ":", old_state, "->", new_state)
                cv.notify()
                cv.release()
        self.thrift_client.set_function_module_state_changed_callback(function_module_changed_callback)
        if self.do_target_search: 
            self.searchTargets()
        else: 
            # register projector to coordinate system
            self.thrift_client.FunctionModuleSetProperty(module_id, "runMode", "1")
            print("Register projector to coordinate system...")
            cv.acquire()
            print("Calculate transformation..")
            self.thrift_client.FunctionModuleRun(module_id)
            cv.wait()
            cv.release()
            state = self.thrift_client.FunctionModuleGetProperty(module_id, "state")
            if state != "1":  # idle
                print("Function module is not in idle state, hence an error has occured.")
                # sys.exit(1)
            else:
                res = self.thrift_client.FunctionModuleGetProperty(module_id, "result.averageDistance");
                #Activate reference object only if the calculated transformation was succesfully
                reference_object = self.thrift_client.GetReferenceobject(reference_object_name)
                reference_object.activated = True
                self.thrift_client.SetReferenceobject(reference_object)
                print("Finished to register projector (aveDist:",res,")\n")
                allCoordinateSystems = self.thrift_client.GetCoordinatesystemList()
                print("Available coordinate systems:", allCoordinateSystems)
                print("Show result of point search for 5 seconds\n")
                thrift_client.FunctionModuleSetProperty(module_id,"showAllRefPts","1")

    def searchTargets(self):
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
        reference_object = self.thrift_client.GetReferenceobject(reference_object_name)
        found_all = True
        for i in range(0, len(reference_object.refPointList)):
            resultPath = "result.tracePoints." + str(i)
            if self.thrift_client.FunctionModuleGetProperty(module_id, resultPath + ".found") != "true":
                found_all = False
                break
        if not found_all:
            print("Not all points have been found!")
        else:
            print("All points have been found.")

    def setUpProjector(self):
        self.clientServerConnect()
        self.activateProjector()
        self.transferLicense()
        self.defineCoordinateSystem()

    def createCircle(self,x,y,r):
        name = self.projection_group + "/my_circle"
        circle = zlp.create_circle(x, y, r, name)
        circle.activated = True
        circle.coordinateSystemList = self.coordinate_system
        self.thrift_client.SetCircleSegment(circle)
    
    def createPolyline(self):
        name = self.projection_group + "/my_polyline"
        polyline = zlp.create_polyline(name)
        linestringA = [ zlp.create_3d_point(-100, -100),
                        zlp.create_3d_point(+100, -100),
                        zlp.create_3d_point(+100, +100) ]
        linestringB = [ zlp.create_3d_point(-200, -200),
                        zlp.create_3d_point(+200, -200),
                        zlp.create_3d_point(+200, +200) ]
        polyline.polylineList = [linestringA, linestringB]
        polyline.activated = True
        polyline.coordinateSystemList = self.coordinate_system
        self.thrift_client.SetPolyLine(polyline)

    def createArc(self,x,y,r,startAngle,endAngle):
        name = self.projection_group + "/my_arc"
        arc = zlp.create_circle(x, y, r, name)
        arc.startAngle = startAngle
        arc.endAngle = endAngle
        arc.activated = True
        arc.coordinateSystemList = self.coordinate_system
        self.thrift_client.SetCircleSegment(arc)

    def createText(self,x,y,text):
        name = self.projection_group + "/my_text"
        text = zlp.create_text_element(x, y, text, name, 100)
        text.charSpacing = 100
        text.angle = 25
        text.activated = True
        text.coordinateSystemList = self.coordinate_system
        self.thrift_client.SetTextElement(text)

    def startProjection(self):
        self.thrift_client.TriggerProjection()

    def cleanProjection(self):
        circle.activated   = False
        polyline.activated = False
        arc.activated      = False
        text.activated     = False
        self.thrift_client.SetCircleSegment(circle)
        self.thrift_client.SetPolyLine(polyline)
        self.thrift_client.SetCircleSegment(arc)
        self.thrift_client.SetTextElement(text)
        self.startProjection()                          # TODO revise this

    