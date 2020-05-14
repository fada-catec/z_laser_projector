#!/usr/bin/env python3

from zlaser_sdk_ros.projector_manager import ProjectorManager
from zlaser_sdk_ros.zlp import CoordinateSystemParameters, ProjectionElementParameters
from std_msgs.msg import Bool, String, Float64, Int16


lic_path = "/lic/1900027652.lic"
projector_IP = "192.168.10.10"
server_IP = "192.168.10.11"
connection_port = 9090
projector = ProjectorManager(projector_IP, server_IP, connection_port)

class request(object):
    name_cs      = String('MyCoordinateSystem_1')
    distance     = Float64(1410.0)
    x1           = Float64(280.0)
    y1           = Float64(10.0)
    x2           = Float64(-285.0)
    y2           = Float64(10.0)
    x3           = Float64(-285.0)
    y3           = Float64(-384.0)
    x4           = Float64(280.0)
    y4           = Float64(-384.0)
    T1_x         = Float64(0.0)
    T1_y         = Float64(0.0)
    scale_factor = Int16(1)

req = request()
print(req.name_cs.data)

def main():
    
    s,m = projector.client_server_connect()
    if not s:
        print("Client server is not connected.")
    
    s,m = projector.activate()
    if not s:
        print("Projector is not activated.")
    
    print("Projector connected and activated.")

    s,m = projector.load_license(lic_path)
    if not s: 
        print("License not loaded.")
    
    s,m = projector.geotree_operator_create()
    if not s:
        print("Geotree operator not created.")
    
    print("License loadded and geotree operator created.")

    cs_params = CoordinateSystemParameters(req)

    s,m = projector.define_coordinate_system(cs_params)
    if not s:
        print("Coordinate system not defined.")
    
    s,m = projector.show_coordinate_system(5)
    if not s:
        print("Coordinate system not showed.")
    
    s,m = projector.cs_axes_create(cs_params)
    if not s:
        print("Coordinate system axes not created.")
        
    print("Coordinate system defined and showed.")

if __name__ == '__main__':
    main()