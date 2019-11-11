#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import threading
import os
from zlaser_sdk_ros import zlp

do_point_search = False

# connect to projector
thrift_client = zlp.ThriftClient()
thrift_client.connect("192.168.10.11", 9090)

# update license information
try:
    print("Transferring license same file...")
    license_path = os.path.abspath("/home/catec/Pendrive_ZLaser/1900027652.lic")
    print (license_path)
    license_file = os.path.basename(license_path)
    print (license_file)
    thrift_client.transfer_file(license_path, license_file, True)

except zlp.thrift_interface.CantWriteFile as e:
    print(e.why, e.fileName)

except FileNotFoundError:
    print("File not found!")

thrift_client.LoadLicense(license_file)

# activate projector
projectors = thrift_client.scan_projectors("192.168.10.10")
projector = projectors[0]
thrift_client.activate_projector(projector)

# define coordinate system
allCoordinateSystems = thrift_client.GetCoordinatesystemList()
print("Available coordinate systems:", allCoordinateSystems)

reference_object = zlp.create_reference_object()
reference_object_name = "RefObject"

print("Create the reference object...")

reference_object.name = reference_object_name
reference_object.refPointList = [zlp.create_reference_point("T1", 0, 0),
                                 zlp.create_reference_point("T2", 2000, 0),
                                 zlp.create_reference_point("T3", 0, 2000),
                                 zlp.create_reference_point("T4", 2000, 2000)]

# set global crosssize for all reference points
crossSize = zlp.create_2d_point(50,50)

# - define coordinates in system of factory calibration wall [mm]
# - activate reference point to use for transformation
reference_object.refPointList[0].tracePoint.x = 0
reference_object.refPointList[0].tracePoint.y = 0
reference_object.refPointList[0].distance = 3533.4
reference_object.refPointList[0].activated = True
reference_object.refPointList[0].crossSize = crossSize

reference_object.refPointList[1].tracePoint.x = 1000
reference_object.refPointList[1].tracePoint.y = 0
reference_object.refPointList[1].distance = 3533.4
reference_object.refPointList[1].activated = True
reference_object.refPointList[1].crossSize = crossSize

reference_object.refPointList[2].tracePoint.x = 0
reference_object.refPointList[2].tracePoint.y = 1000
reference_object.refPointList[2].distance = 3533.4
reference_object.refPointList[2].activated = True
reference_object.refPointList[2].crossSize = crossSize

reference_object.refPointList[3].tracePoint.x = 1000
reference_object.refPointList[3].tracePoint.y = 1000
reference_object.refPointList[3].distance = 3533.4
reference_object.refPointList[3].activated = True
reference_object.refPointList[3].crossSize = crossSize

reference_object.coordinateSystem = "Aeropaint_CS"
reference_object.projectorID = projector

thrift_client.SetReferenceobject(reference_object)

# create function module to register the projector to coordinate system
module_id = ""
try:
    module_id = thrift_client.FunctionModuleCreate("zFunctModRegister3d", "3DReg")

except zlp.thrift_interface.FunctionModuleClassNotRegistered as e:
    print("FunctionModuleClassNotRegistered: " + e.which)
    sys.exit(1)
except zlp.thrift_interface.FunctionModulePropertyBranchAlreadyInUse as e:
    print("FunctionModulePropertyBranchAlreadyInUse: " + e.branchName)
    sys.exit(1)
except zlp.thrift_interface.FunctionModuleClassNotLicensed as e:
    print("FunctionModuleClassNotLicensed: " + e.which)
    sys.exit(1)

thrift_client.FunctionModuleSetProperty(module_id, "referenceData", reference_object_name)

cv = threading.Condition()
def function_module_changed_callback(module_id, old_state, new_state):
    if new_state != zlp.thrift_interface.FunctionModuleStates.RUNNING:
        cv.acquire()
        print("Function module stopped running.")
        print("Module", module_id, ":", old_state, "->", new_state)
        cv.notify()
        cv.release()

thrift_client.set_function_module_state_changed_callback(function_module_changed_callback)

if not do_point_search:
    print("Do no point search. Use given coordinates.")

# register projector to coordinate system
thrift_client.FunctionModuleSetProperty(module_id, "runMode", "1")
print("Register projector to coordinate system...")

cv.acquire()
print("Calculate transformation..")
thrift_client.FunctionModuleRun(module_id)
cv.wait()
cv.release()

state = thrift_client.FunctionModuleGetProperty(module_id, "state")
if state != "1":  # idle
    print("Function module is not in idle state, hence an error has occured.")
    sys.exit(1)
else:
    res = thrift_client.FunctionModuleGetProperty(module_id, "result.averageDistance")
    #Activate reference object only if the calculated transformation was succesfully
    reference_object = thrift_client.GetReferenceobject(reference_object_name)
    reference_object.activated = True
    thrift_client.SetReferenceobject(reference_object)
    print("Finished to register projector (aveDist:",res,")\n")

    allCoordinateSystems = thrift_client.GetCoordinatesystemList()
    print("Available coordinate systems:", allCoordinateSystems)

    print("Show result of point search for 5 seconds\n")
    thrift_client.FunctionModuleSetProperty(module_id,"showAllRefPts","1")
    time.sleep(5)

# clean-up
thrift_client.RemoveGeoTreeElem(reference_object_name)
thrift_client.FunctionModuleRelease(module_id)
thrift_client.deactivate_projector(projector)

thrift_client.disconnect()
