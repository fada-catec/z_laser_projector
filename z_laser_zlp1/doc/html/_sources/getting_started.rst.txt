###############
Getting started
###############

The z_laser_projector package is a ROS wrapper for `ZLP1 <(https://z-laser.com/en/product/laser-projector/zlp1/)>`__
Z-LASER Projector, that makes it fully compatible with all available sensors and devices running over the ROS-Industrial project. 
This package provides libraries that allow the user to operate the device and simplify the task of developing further advanced 
features, such as augmented reality applications. 

The ZLP1 is an eye-safe laser projector (laser class 2M). It is used to project desired objects and figures as an optical 
guidance system for human operator tasks in industrial production processes like pick-and-place, logistics, and workstations, 
endeavouring to enlarge and optimize the production and workflow. 

.. image:: /_static/ZLP1.png
   :scale: 30
   :align: center

The collection of rosservices supplied by the package wraps the functionalities of the device helping with the development 
of advanced applications in factories facilities, and motivating the ROS-Industrial community growth.


:Author: `Rafael Luque <rluque@catec.aero>`__

:Author: `Inés M. Lara <imlara@catec.aero>`__

:Maintainers: Rafael Luque, Inés M. Lara

:Repository: https://github.com/fada-catec/z_laser_projector.git

:ROS Wiki: http://wiki.ros.org/z_laser_projector

:Version: 1.0

:License: `Apache 2.0 <https://www.apache.org/licenses/LICENSE-2.0>`__

******
Set up
******

1. Create a workspace and clone this repo:

   ::

         mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
         git clone https://github.com/fada-catec/z_laser_projector.git
         cd ~/catkin_ws && catkin_make

2. Don't forget to source workspace:

   ::

         source ~/catkin_ws/devel/setup.bash

3. Set the correct network to reach projector.

   Connect projector to PC and set a network on IP ``192.168.10.9`` and submask ``255.255.255.0``

   To consider:

   -  The internal zService is running inside the projector under ``192.168.11.11`` on port ``9090``.

   -  Projector is always reachable under ``192.168.10.10``.

************
Dependencies
************

-  The code have been tested for `ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`__ on **Ubuntu 18.04**.

-  This software runs on **python 3**. Install dependencies by using ``pip3``

   ::

         sudo apt-get install python3-pip
         pip3 install thriftpy
         pip3 install numpy

*****
Usage
*****

You can launch the ROS node included in this package, which opens ROS services to operate de projector. 

   ::

         roslaunch z_laser_projector projector_zlp1.launch

It is possible to change rosparameters and get them by the ``load_user_coordinate_system`` argument in launch file.

Or if you perfer to include some projector functionalities into your custom node, import the libraries instead:

   ::

         #!/usr/bin/env python3
         from z_laser_projector.zlp_projector_manager import ZLPProjectorManager
         from z_laser_projector.zlp_utils import CoordinateSystemParameters, ProjectionElementParameters


*****
Tests
*****

Run automatic integration tests:

   ::

         rostest z_laser_projector connection_test.test
         rostest z_laser_projector define_coordinate_system_test.test

****
Help
****

-  Ines M. Lara - imlara@catec.aero
-  Rafael Luque - rluque@catec.aero
-  Other community or team contact