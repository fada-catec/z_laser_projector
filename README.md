![ZLASER](./z_laser_projector/images/ZLASER.png)![FADA](./z_laser_projector/images/FADA.png)![CATEC](./z_laser_projector/images/CATEC.png)

# ROS z_laser_projector Stack

[![Build Status](https://travis-ci.com/fada-catec/z_laser_projector.svg?token=euTp3jtyEts1qcm7iWeV&branch=melodic)](https://travis-ci.com/fada-catec/z_laser_projector)
[![License](https://img.shields.io/badge/License-Apache%202-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![codecov](https://codecov.io/gh/fada-catec/z_laser_projector/branch/melodic/graph/badge.svg)](https://codecov.io/gh/fada-catec/z_laser_projector)

The [z_laser_projector stack](http://wiki.ros.org/z_laser_projector) is a set of tools that allow the user to operate the Z-LASER Projector [ZLP1](https://z-laser.com/en/product/laser-projector/zlp1/) and simplify the task of developing further advanced features, such as augmented reality applications. The stack provides a ROS API to control the device via topics and services, a visualizer for laser projections and a graphical interface.


The code have been tested for **ROS Melodic** on **Ubuntu 18.04**.

This project is licensed under the terms of the [**Apache 2.0**](https://www.apache.org/licenses/LICENSE-2.0) license.

## Set up

1. Create a workspace and clone this repo:

          mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
          git clone https://github.com/fada-catec/z_laser_projector.git
          cd ~/catkin_ws && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_VERSION=3.6
          source ~/catkin_ws/devel/setup.bash
     
2. Set the correct network to reach projector:

     Connect projector to PC and set a network on IP range `192.168.10.X` and submask `255.255.255.0`

     To consider:

     - The internal zService is running inside the projector under `192.168.11.11` on port `9090`.

     - Projector is always reachable under `192.168.10.10`. 

3. A device-key license to check the purchase of the device is always required to use the ZLP software. Please, contact your Z-LASER supplier to get a valid license. Save the license file under package directory `z_laser_zlp1/lic` folder. Then, indicate the license file name on the configuration file `z_laser_zlp1/config/communication_settings.yaml`.

## Dependencies
   
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

- This software runs on **python 3**. You can install dependencies:

          cd ~/catkin_ws
          rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

     or

          sudo apt-get install python3-thriftpy
          sudo apt-get install python3-numpy
          sudo apt-get install python3-scipy
          sudo apt-get install python3-ezdxf
          sudo apt-get install python3-pyqt5
          sudo apt-get install python3-nose

- Install pynput

          sudo apt-get install python3-pip
          pip3 install pynput

## Usage

### Projector Node

You can launch the [projector node](https://github.com/fada-catec/z_laser_projector/z_laser_zlp1) and operate the device throughout the ROS API provided. Set argument `load_user_coorinate_system` to true for loading initial values of the reference system from configuration files.

     roslaunch z_laser_zlp1 z_laser_zlp1.launch

### Projector Node + Visualizer

You can launch the projector node together with [visualizer](https://github.com/fada-catec/z_laser_projector/z_laser_viz) node. Note: the projector node launch is included in `z_laser_viz`:

     roslaunch z_laser_viz z_laser_viz.launch

### Graphical User Interface

Additionally, you can also launch the [GUI](https://github.com/fada-catec/z_laser_projector/z_laser_viz) node.

     roslaunch z_laser_gui z_laser_gui.launch

### DXF Reader

We have created a functionality capable of reading and interpreting a DXF graphic file. By executing the `/zlp_dxf_reader` node together with the projector node, the elements found in a .dxf file are loaded. Call `~/projection_start` service to project them. Specify custom .dxf file name as argument (this file should be placed in `/dxf` folder from z_laser_zlp1 package). 

     roslaunch z_laser_zlp1 z_laser_dxf_reader.launch dxf_file_name:=dxf_test

NOTE: figures' units are read as millimetres

### Libraries

On the other hand, if you prefer to include some projector functionalities into your custom node or application, import the libraries instead:

     #!/usr/bin/env python3
     import z_laser_zlp1.zlp_projector_manager

## Test

Run unit tests and integration tests automatically:

     catkin_make run_tests -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_VERSION=3.6

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
