![ZLASER](./images/ZLASER.png)![FADA](./images/FADA.png)![CATEC](./images/CATEC.png)

[//]: <> (<img src="https://z-laser.com/wp-content/uploads/zLaserLogo.png" width="142" height="40">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<img src="http://www.catec.aero/sites/default/files/FADA.png" width="170" height="50">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<img src="http://www.catec.aero/sites/default/files/Logo_CATEC_1.png" width="138" height="50">)

# z_laser_projector

This package is a ROS wrapper to control the Z-LASER Projector [ZLP1]((https://z-laser.com/en/product/laser-projector/zlp1/)) via ROS using Z-LASER SDK.

The code have been tested for **ROS Melodic** on **Ubuntu 18.04**.

This project is licensed under the terms of the [**Apache 2.0**](https://www.apache.org/licenses/LICENSE-2.0) license.

## Set up

1. Create a workspace and clone this repo

          mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
          git clone https://github.com/fada-catec/z_laser_projector.git
          cd ~/catkin_ws && catkin_make
     
2. Don't forget to source workspace:

          source ~/catkin_ws/devel/setup.bash

3. Set the correct network to reach projector

     Connect projector to PC and set a network on IP `192.168.10.9` and submask `255.255.255.0`

     To consider:

     - The internal zService is running inside the projector under `192.168.11.11` on port `9090`.

     - Projector is always reachable under `192.168.10.10`. 

## Dependencies
   
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

- This software runs on **python 3**. Install dependencies by using `pip3`

          sudo apt-get install python3-pip
          pip3 install thriftpy
          pip3 install numpy

## Usage

You can launch the ROS node included in this package, which opens ROS services to operate de projector. 

     roslaunch zlaser_sdk_ros projector_zlp1.launch

If you perfer to include some projector functionalities into your custom node, import the librearies instead:

     #!/usr/bin/env python3
     from zlaser_sdk_ros.projector_manager import ProjectorManager
     from zlaser_sdk_ros.utils import CoordinateSystemParameters, ProjectionElementParameters

## Known issues

* RecursionError: maximum recursion depth exceeded while calling a Python object: this means that ethernet connection have not been stablished. Restart projector, wait for green LED's and connect again.
* Error processing request: FunctionModuleNotExistent(fModUID=''): this means that license could not have registered zFunctModRegister3d or that you are trying to create a coordinate system that already exists.
* Error processing request: 'ProjectorManager' object has no attribute 'projector_id': this means that projector instance has not been created. Call to `connect or setup service` to create it first.
* Error processing request: ServiceInterfaceHandler::TriggerProjection() bv::InvalidState(What: Keine Projektoren verbunden!): license problems. Close node and restart again. Perform call in this order: connect, load_license

## Code API

[Topics, Services](http://wiki.ros.org/z_laser_projector#Code_API)

Read documentation `z_laser_projector/doc/html/index.html`

## Acknowledgement

***
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 

## Help

* Ines M. Lara - imlara@catec.aero
* Rafael Luque - rluque@catec.aero
* Other community or team contact