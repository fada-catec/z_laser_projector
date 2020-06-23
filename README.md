![ZLASER](./images/ZLASER.png)![FADA](./images/FADA.png)![CATEC](./images/CATEC.png)

[//]: <> (<img src="https://z-laser.com/wp-content/uploads/zLaserLogo.png" width="142" height="40">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<img src="http://www.catec.aero/sites/default/files/FADA.png" width="170" height="50">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<img src="http://www.catec.aero/sites/default/files/Logo_CATEC_1.png" width="138" height="50">)

# z_laser_projector - ROS Wrapper for ZLP1 Z-LASER Projector 

This package is used to control the Z-LASER Projector ZLP1 via ROS using Z-LASER SDK.

This code have been tested for **ROS Melodic** on **Ubuntu 18.04**.

[More information](https://z-laser.com/en/product/laser-projector/zlp1/)

This project is licensed under the terms of the [**Apache 2.0**](https://www.apache.org/licenses/LICENSE-2.0) license.

## Installation Instructions

### Step 1: Install the ROS distribution
   
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

### Step 2: Install dependencies

- This software runs on **python 3**. Install dependencies by using `pip3`
     ```
     sudo apt-get install python3-pip
     ```
- thriftpy
     ```
     pip3 install thriftpy
     ```
- numpy
     ```
     pip3 install numpy
     ```

### Step 3: Install z_laser_projector ROS package

- Create a [catkin](http://wiki.ros.org/catkin#Installing_catkin) workspace
     ```
     mkdir -p ~/catkin_ws/src
     cd ~/catkin_ws/
     catkin_make
     ```
- Clone and compile the latest z_laser_projector ROS package
     ```
     cd ~/catkin_ws/src/
     git clone https://github.com/fada-catec/z_laser_projector.git
     cd ~/catkin_ws
     catkin_make
     ```
- Don't forget to source workspace:
     ```
     source ~/catkin_ws/devel/setup.bash
     ```

## Usage Instructions

### Set the correct network to reach projector

Connect projector to PC and set a network on IP `192.168.10.9` and submask `255.255.255.0`

To consider:

- The internal zService is running inside the projector under `192.168.11.11` on port `9090`.

- Projector is always reachable under `192.168.10.10`. 

### Start the projector node

You can launch the ROS node included in this package, which opens ROS services to operate de projector. 

```
roslaunch zlaser_sdk_ros projector_zlp1.launch
```

### Package libraries

If you perfer to include some projector functionalities into your custom node, import the librearies instead:
```
#!/usr/bin/env python3
from zlaser_sdk_ros.projector_manager import ProjectorManager
from zlaser_sdk_ros.utils import CoordinateSystemParameters, ProjectionElementParameters
```

## Available services

* `/projector_srv/connect`: connect to thrift server of ZLP-Service and activate the device.
* `/projector_srv/disconnect`:  deactivate projector and disconnect from service.
* `/projector_srv/load_license`: transfer license file to service and check correct loading.
* `/projector_srv/setup`: pack basic services: connect to service, activate projector and transfer license.
* `/projector_srv/projection_start`: start projection on the surface of figures associated to the __current__ coordinate system (see /projector_srv/set_cs service).
* `/projector_srv/projection_stop`: stop all figures projection.
* `/projector_srv/man_def_cs`: define a new reference system, stating the points coordinates manually by the user.
* `/projector_srv/cs_list`: get the list of defined coordinate systems.
* `/projector_srv/set_cs`: set the current operating coordinate system which some services as 'add_shape' or ‘projection_start’ automatically apply to. The rest of defined systems stay on background until any is set again.
* `/projector_srv/show_current_cs`: project reference points and origin axis of current coordinate system.
* `/projector_srv/remove_coord_sys`: remove current coordinate system.
* `/projector_srv/add_shape`: define properties of a new figure (line, circle, etc.) and add it to the figures list associated to the current coordinate system.
* `/projector_srv/hide_shape`: hide specific figure from current coordinate system.
* `/projector_srv/unhide_shape`: unhide specific figure from current coordinate system.
* `/projector_srv/remove_shape`: remove specific figure from current coordinate system.

## Known issues
* RecursionError: maximum recursion depth exceeded while calling a Python object: this means that ethernet connection have not been stablished. Restart projector, wait for green LED's and connect again.
* Error processing request: FunctionModuleNotExistent(fModUID=''): this means that license could not have registered zFunctModRegister3d or that you are trying to create a coordinate system that already exists.
* Error processing request: 'ProjectorManager' object has no attribute 'projector_id': this means that projector instance has not been created. Call to `connect or setup service` to create it first.
* Error processing request: ServiceInterfaceHandler::TriggerProjection() bv::InvalidState(What: Keine Projektoren verbunden!): license problems. Close node and restart again. Perform call in this order: connect, load_license

## Acknowledgement

![ROSIN](./images/ROSIN.png)

Supported by ROSIN - ROS-Industrial Focused Technical Projects (FTP).  
More information: [rosin-project.eu](http://rosin-project.eu)

## Help

* Ines M. Lara - imlara@catec.aero
* Rafael Luque - rluque@catec.aero
* Other community or team contact