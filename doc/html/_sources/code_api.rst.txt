.. ros:package:: z_laser_projector

########
CODE API
########

.. contents:: Table of Contents
  :local:
  :depth: 1

********
Messages
********

.. ros:message:: Line

  :msg_param group_name: name of the projection elements group to which the line belongs
  :msg_paramtype group_name: :ros:msg:`string`
  :msg_param shape_id: name of the line element to define
  :msg_paramtype shape_id: :ros:msg:`string`
  :msg_param x: x position of the line's starting point
  :msg_paramtype x: :ros:msg:`float64`
  :msg_param y: y position of the line's starting point
  :msg_paramtype y: :ros:msg:`float64`
  :msg_param angle: line slope
  :msg_paramtype angle: :ros:msg:`float64`
  :msg_param length: line length
  :msg_paramtype length: :ros:msg:`float64`

  .. raw:: html

    <embed>
        <li><a href="msg/Line.html">Line.msg</a></li>
    </embed>

********
Services
********

.. ros:service:: CoordinateSystem

  :req_param name: coordinate system name
  :req_paramtype name: :ros:msg:`string`
  :req_param distance: distance between the projector and the projection surface
  :req_paramtype distance: :ros:msg:`float64`
  :req_param P1: x,y position of P1 point from projector reference system {P}
  :req_paramtype P1: :ros:msg:`geometry_msgs/Point`
  :req_param P2: x,y position of P2 point from projector reference system {P}
  :req_paramtype P2: :ros:msg:`geometry_msgs/Point`
  :req_param P3: x,y position of P3 point from projector reference system {P}
  :req_paramtype P3: :ros:msg:`geometry_msgs/Point`
  :req_param P4: x,y position of P4 point from projector reference system {P}
  :req_paramtype P4: :ros:msg:`geometry_msgs/Point`
  :req_param T1: x,y position of T1 point from user reference system {T}
  :req_paramtype T1: :ros:msg:`geometry_msgs/Point`
  :req_param resolution: resolution of user coordinate system {T}
  :req_paramtype resolution: :ros:msg:`float64`
  :resp_param T: return list of user reference points [T1,T2,T3,T4]
  :resp_paramtype T: :ros:msg:`geometry_msgs/Point[]`
  :resp_param cs_created: return value response. True for success, False otherwise
  :resp_paramtype cs_created: :ros:msg:`bool`
  :resp_param message: message response for error or success identification
  :resp_paramtype message: :ros:msg:`string`

  .. raw:: html

    <embed>
        <li><a href="srv/CoordinateSystem.html">CoordinateSystem.srv</a></li>
    </embed>

.. ros:service:: CoordinateSystemList

  :resp_param success: return value response. True for success, False otherwise
  :resp_paramtype success: :ros:msg:`bool`
  :resp_param message: message response for error or success identification
  :resp_paramtype message: :ros:msg:`string`
  :resp_param cs_list: list of defined coordinate systems
  :resp_paramtype cs_list: :ros:msg:`string[]`

  .. raw:: html

    <embed>
        <li><a href="srv/CoordinateSystemList.html">CoordinateSystemList.srv</a></li>
    </embed>

.. ros:service:: CoordinateSystemName

  :req_param name: coordinate system name
  :req_paramtype name: :ros:msg:`string`
  :resp_param success: return value response. True for success, False otherwise
  :resp_paramtype success: :ros:msg:`bool`
  :resp_param message: message response for error or success identification
  :resp_paramtype message: :ros:msg:`string`

  .. raw:: html

    <embed>
        <li><a href="srv/CoordinateSystemName.html">CoordinateSystemName.srv</a></li>
    </embed>

.. ros:service:: CoordinateSystemShow

  :req_param secs: number of projection seconds
  :req_paramtype secs: :ros:msg:`int16`
  :resp_param success: return value response. True for success, False otherwise
  :resp_paramtype success: :ros:msg:`bool`
  :resp_param message: message response for error or success identification
  :resp_paramtype message: :ros:msg:`string`

  .. raw:: html

    <embed>
        <li><a href="srv/CoordinateSystemShow.html">CoordinateSystemShow.srv</a></li>
    </embed>

.. ros:service:: ProjectionElement

  :req_param shape_type: type of projection element
  :req_paramtype shape_type: :ros:msg:`string`
  :req_param group_name: name of the projection elements group to which the projection element belongs
  :req_paramtype group_name: :ros:msg:`string`
  :req_param shape_id: projection element name
  :req_paramtype shape_id: :ros:msg:`string`
  :resp_param success: return value response. True for success, False otherwise
  :resp_paramtype success: :ros:msg:`bool`
  :resp_param message: message response for error or success identification
  :resp_paramtype message: :ros:msg:`string`

  .. raw:: html

    <embed>
        <li><a href="srv/ProjectionElement.html">ProjectionElement.srv</a></li>
    </embed>


********
Modules
********

.. toctree::
  :maxdepth: 1

  zlp_core
  zlp_utils
  zlp_projector_manager
  zlp_projector_ros