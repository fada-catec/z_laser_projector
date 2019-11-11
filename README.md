# zlaser_sdk_ros

This packages is used to control Laser Projector ZLP1 via ROS using ZLASER SDK.

### Usage ###
Once the setup is done it is possible to use libraries from this repo to manage the projector.

Import projector manager on your script and feel free to use it:
```
#!/usr/bin/env python3

from zlaser_sdk_ros.projector_manager import ProjectorManager
```

### Setup ###

* Install dependencies
* Clone this repo 
* Compile

        catkin_make

* Source your workspace
* Connect projector to PC and set a network on IP `192.168.10.9` and submask `255.255.255.0`
* The server is running inside the projector under `192.168.11.11` on port `9090`.
* Projector is always reachable under `192.168.10.10`. 

### Dependencies ###

* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Python 3.5 under `#!/usr/bin/env python3` and pip3

        sudo apt-get install python3-pip

* thriftpy:

        pip3 install thriftpy

### Help ###

* Ines M. Lara - imlara@catec.aero
* Other community or team contact