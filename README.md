![ZLASER](./images/ZLASER.png)![FADA](./images/FADA.png)![CATEC](./images/CATEC.png)

[//]: <> (<img src="https://z-laser.com/wp-content/uploads/zLaserLogo.png" width="142" height="40">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<img src="http://www.catec.aero/sites/default/files/FADA.png" width="170" height="50">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<img src="http://www.catec.aero/sites/default/files/Logo_CATEC_1.png" width="138" height="50">)

# z_laser_projector - ROS Wrapper for ZLP1 Z-LASER Projector 

This package is used to control the Z-LASER Projector ZLP1 via ROS using Z-LASER SDK. 

[More information](https://z-laser.com/en/product/laser-projector/zlp1/)

## Installation Instructions

The following instructions are written for ROS Melodic on **Ubuntu 18.04**.

### Step 1: Install the ROS distribution
   
   - Install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) on **Ubuntu 18.04**.

### Step 2: Install z_laser_projector ROS package

There are 3 sources to install z_laser_projector from:

* ### Method 1: The ROS distribution

z_laser_projector is available as a debian package of ROS distribution. It can be installed by typing:

```bash
sudo apt-get install ros-melodic-z-laser-projector
```

This will install both z_laser_projector and its dependents, including zlp and projector_manager libraries and Z-LASER SDK.

* ### Method 2: Install z_laser_projector package from Sources

   - Python 3.5 under `#!/usr/bin/env python3`, install extensions by python install packages tool (pip3):
        ```bash
        sudo apt-get install python3-pip
        pip3 install thriftpy
        pip3 install numpy
        ```

   - Create a [catkin](http://wiki.ros.org/catkin#Installing_catkin) workspace
        ```bash
        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws/
        catkin_make
        ```

   - Clone the latest z_laser_projector ROS package from [here](https://github.com/blablabla/releases) into 'catkin_ws/src/'
        ```bashrc
        cd ~/catkin_ws/src/
        git clone https://blablabla.git
        ```
   
   - Compile:
        ```bash
        cd ~/catkin_ws
        catkin_make
        ```

   - Source workspace:
        ```bash
        source ~/catkin_ws/devel/setup.bash
        ```

   - Connect projector to PC and set a network on IP `192.168.10.9` and submask `255.255.255.0`

   - The server is running inside the projector under `192.168.11.11` on port `9090`.

   - Projector is always reachable under `192.168.10.10`. 

* ### Method 3: Install libraries as python extensions

Do not install the complete ROS package?? and use the libraries in your own scripts.
It is possible to import the libraries developed in this repository (projector_manager) on your script and feel free to handle the projector.
```
pip3 install projector_manager
```

## Usage Instructions

### Start the projector node

In case of complete ROS package installation, you can launch the ros node included in this package to connect to projector:

```bash
roslaunch zlaser_sdk_ros projector_zlp1.launch
```
This will launch all laser projector services.

### Package libraries

In order to only use the libraries in another package or script after install de python extension [projector_manager](#method-3:-install-libraries-as-python-extensions) or after install the z_laser_projector package, you can import the libraries:
```
#!/usr/bin/env python3
from zlaser_sdk_ros.projector_manager import ProjectorManager
from zlaser_sdk_ros.utils import CoordinateSystemParameters, ProjectionElementParameters
```

## Available services

* `/projector_srv/connect`: connect to service and activate projector.  
* `/projector_srv/disconnect`:  disconnect from service and deactivate projector.
* `/projector_srv/load_license`: send license to service.
* `/projector_srv/setup`: connect to service, activate projector and send license.
* `/projector_srv/projection_start`: start projection of elements from the __current__ coordinate system (see set_cs service).
* `/projector_srv/projection_stop`: stop projection.
* `/projector_srv/man_def_cs`: define a new coordinate system manually, stating points coordinates by the user.
* `/projector_srv/cs_list`: get list of defined coordinate systems.
* `/projector_srv/set_cs`: set the current coordinate system, because some services as 'add_shape' apply automatically to the current coordinate system. It means that the set coordinate system become in the coordinate system to operate with and the rest stay on background until any is set again.
* `/projector_srv/show_current_cs`: project current coordinate system.
* `/projector_srv/remove_coord_sys`: remove current coordinate system.
* `/projector_srv/add_shape`: define properties of a new shape (line, circle, ...) and add it to the current coordinate system.
* `/projector_srv/hide_shape`: hide specific shape from current coordinate system.
* `/projector_srv/unhide_shape`: unhide specific shape from current coordinate system.
* `/projector_srv/remove_shape`: remove specific shape from current coordinate system.

### Known issues ###
* RecursionError: maximum recursion depth exceeded while calling a Python object: this means that ethernet connection have not been stablished. Restart projector, wait for green LED's and connect again.
* Error processing request: FunctionModuleNotExistent(fModUID=''): this means that license could not have registered zFunctModRegister3d or that you are trying to create a coordinate system that already exists.
* Error processing request: 'ProjectorManager' object has no attribute 'projector_id': this means that projector instance has not been created. Call to `connect or setup service` to create it first.
* Error processing request: ServiceInterfaceHandler::TriggerProjection() bv::InvalidState(What: Keine Projektoren verbunden!): license problems. Close node and restart again. Perform call in this order: setup, cs, project

### Acknowledgement

![ROSIN](./images/ROSIN.png)

Supported by ROSIN - ROS-Industrial Focused Technical Projects (FTP).  
More information: [rosin-project.eu](http://rosin-project.eu)

### Help ###

* Ines M. Lara - imlara@catec.aero
* Rafael Luque - rluque@catec.aero
* Other community or team contact










## Getting started

- [Install](#setup) the ZLP1 ROS Wrapper.
- For more information, check out our [ROS documentation]().


### Dependencies

- Ubuntu 18.04.
- [ZLP-Suite Linux SDK]().
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).

The z_laser_projector is a catkin package and depends on the following ROS packages:

   - catkin
   - roscpp
   - rospy
   - std_msgs
   - tf
   - message_generation


### Setup

* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

* Install dependencies:
        
  - Python 3.5 under `#!/usr/bin/env python3`, python install packages tool (pip3) and extensions (thriftpy, numpy):
        
        sudo apt-get install python3-pip

        pip3 install thriftpy

        pip3 install numpy

* Clone the repository:
   
        cd ~/catkin_ws/src 
        git clone https://github.com/....git

* Compile:

        cd ~/catkin_ws
        catkin_make

* Source your workspace:
        
        source ./devel/setup.bash

* Connect projector to PC and set a network on IP `192.168.10.9` and submask `255.255.255.0`

* The server is running inside the projector under `192.168.11.11` on port `9090`.

* Projector is always reachable under `192.168.10.10`. 


### Usage

Once the setup is done it is possible to use libraries from this repository to handle the projector.

Import projector manager on your script and feel free to use it:
```
#!/usr/bin/env python3

from zlaser_sdk_ros.projector_manager import ProjectorManager
```

Or launch the ros node included in this package to connect to projector:

        roslaunch zlaser_sdk_ros projector_zlp1.launch

