![ZLASER](./images/ZLASER.png)![FADA](./images/FADA.png)![CATEC](./images/CATEC.png)

# z_laser_projector

[![Build Status](https://travis-ci.com/fada-catec/z_laser_projector.svg?token=euTp3jtyEts1qcm7iWeV&branch=melodic)](https://travis-ci.com/fada-catec/z_laser_projector)
[![License](https://img.shields.io/badge/License-Apache%202-blue.svg)](https://opensource.org/licenses/Apache-2.0)

The z_laser_projector package is a wrapper that provides a ROS API to control the Z-LASER Projector [ZLP1](https://z-laser.com/en/product/laser-projector/zlp1/) via topics and services, a projections visualizer and a GUI.

<!-- The z_laser_projector package is a wrapper to control the Z-LASER Projector [ZLP1](https://z-laser.com/en/product/laser-projector/zlp1/) via ROS API (topics and services), a projections visualizer and a GUI. -->

The code have been tested for **ROS Melodic** on **Ubuntu 18.04**.

This project is licensed under the terms of the [**Apache 2.0**](https://www.apache.org/licenses/LICENSE-2.0) license.

## Set up

1. Create a workspace and clone this repo

          mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
          git clone https://github.com/fada-catec/z_laser_projector.git
          cd ~/catkin_ws && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_VERSION=3.6

   It is also possible to run unit tests and integration tests automatically

          catkin_make run_tests -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_VERSION=3.6
     
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
          pip3 install pynput
          pip3 install scipy
          pip3 install PyQt5

## Usage

### Projector Node

     roslaunch z_laser_zlp1 z_laser_zlp1.launch

You can simply launch the [projector node](https://github.com/fada-catec/z_laser_projector/z_laser_zlp1) and operate the device throughout the ROS API provided. Add argument `load_user_coorinate_system` to load initial values of the reference system from configuration files.

### Projector Node + Visualizer

     roslaunch z_laser_viz z_laser_viz.launch

You can launch projector node together with [visualizer](https://github.com/fada-catec/z_laser_projector/z_laser_viz) node. In this case it is not necessary to launch projector node separately so that it is included in z_laser_viz.

### Graphical Interface

     roslaunch z_laser_gui z_laser_gui.launch

Additionally, you can also launch the [GUI](https://github.com/fada-catec/z_laser_projector/z_laser_viz) node.

### Libraries

On the other hand, if you prefer to include some projector functionalities into your custom node, import the libraries instead:

     #!/usr/bin/env python3
     import z_laser_zlp1.zlp_projector_manager


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