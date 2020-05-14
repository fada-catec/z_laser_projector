#!/usr/bin/env python3

import sys
import time
from zlaser_sdk_ros.projector_manager import ProjectorManager
from zlaser_sdk_ros.utils import CoordinateSystemParameters, ProjectionElementParameters


lic_path = "./lic/1900027652.lic"
projector_IP = "192.168.10.10"
server_IP = "192.168.10.11"
connection_port = 9090
projector = ProjectorManager(projector_IP, server_IP, connection_port)

cs1_params              = CoordinateSystemParameters()
cs1_params.name         = 'MyCoordinateSystem_1'
cs1_params.d            = 1410.0
cs1_params.x1           = 280.0
cs1_params.y1           = 10.0
cs1_params.x2           = -285.0
cs1_params.y2           = 10.0
cs1_params.x3           = -285.0
cs1_params.y3           = -384.0
cs1_params.x4           = 280.0
cs1_params.y4           = -384.0
cs1_params.T1_x         = 0.0
cs1_params.T1_y         = 0.0
cs1_params.scale_factor = 1

projelemparams_polylineA                       = ProjectionElementParameters()
projelemparams_polylineA.shape_type            = 'polyline'
projelemparams_polylineA.projection_group_name = 'my_group'
projelemparams_polylineA.shape_id              = 'A'
projelemparams_polylineA.x                     = 0.0
projelemparams_polylineA.y                     = 0.0
projelemparams_polylineA.angle                 = 30.0
projelemparams_polylineA.length                = 400.0

def main():
    
    s,m = projector.client_server_connect()
    if not s:
        print("Client server is not connected.")
        sys.exit(0)
    print(m)

    s,m = projector.activate()
    if not s:
        print("Projector is not activated.")
        sys.exit(0)
    print(m)

    print("Projector connected and activated.")

    s,m = projector.load_license(lic_path)
    if not s: 
        print("License not loaded.")
        sys.exit(0)
    print(m)
    
    s,m = projector.geotree_operator_create()
    if not s:
        print("Geotree operator not created.")
        sys.exit(0)
    print(m)

    s,m = projector.define_coordinate_system(cs1_params)
    if not s:
        print("Coordinate system not defined.")
        sys.exit(0)
    print(m)
    
    s,m = projector.show_coordinate_system(5)
    if not s:
        print("Coordinate system not showed.")
        sys.exit(0)
    print(m)
    
    s,m = projector.cs_axes_create(cs1_params)
    if not s:
        print("Coordinate system axes not created.")
        sys.exit(0)
    print(m)

    s,m = projector.create_polyline(projelemparams_polylineA)
    if not s:
        print("Polyline not created.")
        sys.exit(0)
    print(m)

    s,m = projector.show_coordinate_system(5)
    if not s:
        print("Coordinate system not showed.")
        sys.exit(0)
    print(m)

    s,m = projector.cs_axes_unhide()
    if not s:
        print("Coordinate system axes not unhide.")
        sys.exit(0)
    print(m)

    s,m = projector.start_projection()
    if not s:
        print("Projection not started.")
        sys.exit(0)
    print(m)

    time.sleep(4)

    s,m = projector.stop_projection()
    if not s:
        print("Projection not stopped.")
        sys.exit(0)
    print(m)

    s,m = projector.deactivate()
    if not s:
        print("Projector not deactivated.")
        sys.exit(0)
    print(m)
    
    s,m = projector.client_server_disconnect()
    if not s:
        print("Projector not disconnected.")
        sys.exit(0)
    print(m)

if __name__ == '__main__':
    main()