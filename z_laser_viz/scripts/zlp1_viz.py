#! /usr/bin/env python3

import rospy
import math
import numpy as np
from math import sin, cos, pi, radians
from scipy.spatial.transform import Rotation
from pynput import keyboard

from z_laser_zlp1.zlp_keyboard import KeyboardParameters
from z_laser_zlp1.zlp_utils import CoordinateSystemParameters, ProjectionElementParameters

from geometry_msgs.msg import Point, Quaternion, Vector3, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger, TriggerResponse
from z_laser_msgs.msg import Figure
from z_laser_msgs.srv import CoordinateSystem, CoordinateSystemResponse
from z_laser_msgs.srv import CoordinateSystemName, CoordinateSystemNameResponse
from z_laser_msgs.srv import CoordinateSystemShow, CoordinateSystemShowResponse
from z_laser_msgs.srv import CoordinateSystemList, CoordinateSystemListResponse
from z_laser_msgs.srv import ProjectionElement, ProjectionElementResponse

class ZLPVisualizer(object):

    def __init__(self):

        self.cs_marker_array = MarkerArray()
        self.pe_marker_array = MarkerArray()

        self.active_cs = ""
        self.cs_reference = ""
        self.STD_WAIT_TIME = CoordinateSystemParameters().DEFAULT_SHOW_TIME

        self.figures_list = ProjectionElementParameters().figures_list

        self.open_services()
        cs_markers_pub = rospy.Publisher('coord_sys_markers', MarkerArray, queue_size=10)
        pe_markers_pub = rospy.Publisher('proj_elem_markers', MarkerArray, queue_size=10)

        try:
            rate = rospy.Rate(10.0)
            while not rospy.is_shutdown():
                cs_markers_pub.publish(self.cs_marker_array)
                pe_markers_pub.publish(self.pe_marker_array)
                rate.sleep()
            
            rospy.spin()

        except Exception as e:
            rospy.logerr(e)

    def open_services(self):
        
        self.start_proj     = rospy.Service('projection_start', Trigger, self.projection_start_cb)
        self.stop_proj      = rospy.Service('projection_stop', Trigger, self.projection_stop_cb)

        self.manual_cs      = rospy.Service('define_coordinate_system', CoordinateSystem, self.manual_define_coord_sys_cb)
        self.set_cs         = rospy.Service('set_coordinate_system', CoordinateSystemName, self.set_coord_sys_cb)
        self.rem_cs         = rospy.Service('remove_coordinate_system', CoordinateSystemName, self.remove_coord_sys_cb)
        self.show_cs        = rospy.Service('show_active_coordinate_system', CoordinateSystemShow, self.show_coord_sys_cb)

        self.hide_proj_elem    = rospy.Service('hide_projection_element', ProjectionElement, self.hide_proj_elem_cb)
        self.unhide_proj_elem  = rospy.Service('unhide_projection_element', ProjectionElement, self.unhide_proj_elem_cb)
        self.remove_proj_elem  = rospy.Service('remove_projection_element', ProjectionElement, self.remove_proj_elem_cb)

        self.add_proj_elem   = rospy.Subscriber("add_projection_element", Figure, self.add_fig_cb)
        self.monit_proj_elem = rospy.Subscriber("monitor_projection_element", Figure, self.init_keyboard_listener_cb)
        
    def projection_start_cb(self, req):

        if not self.active_cs:
            return TriggerResponse(False, "No Coordinate System set as active.")
        
        for i, marker in enumerate(self.pe_marker_array.markers):
            if marker.ns.find(self.active_cs)>-1:
                self.pe_marker_array.markers[i].action = Marker.ADD
        
        return TriggerResponse(True, "Projection started.")
    
    def projection_stop_cb(self, req):

        for i in range(len(self.pe_marker_array.markers)):
            self.pe_marker_array.markers[i].action = Marker.DELETE

        return TriggerResponse(True, "Projection stopped.")

    def manual_define_coord_sys_cb(self, req):
        for marker in self.cs_marker_array.markers:
            if req.name in marker.ns:
                return CoordinateSystemResponse([], False, "Coordinate System already exists.")

        self.active_cs = req.name

        axis_x_marker, axis_y_marker = self.coord_sys_axes(req)
        self.cs_marker_array.markers.append(axis_x_marker) 
        self.cs_marker_array.markers.append(axis_y_marker) 
        self.cs_marker_array.markers.append(self.coord_sys_frame(req))

        self.cs_reference = "_origin"
        self.timer_secs = self.STD_WAIT_TIME
        self.update_cs_markers()

        return CoordinateSystemResponse([], True, "Coordinate System added manually.")

    def timer_cb(self, timer):

        for i in range(len(self.cs_marker_array.markers)):
            self.cs_marker_array.markers[i].action = Marker.DELETE

        self.update_cs_markers()

    def update_cs_markers(self):

        for marker in self.cs_marker_array.markers:
            if (self.active_cs + self.cs_reference) in marker.ns:
                marker.action = Marker.ADD
        
        if self.cs_reference in ["_origin","_frame"]:
            rospy.Timer(rospy.Duration(self.timer_secs), self.timer_cb, oneshot=True)

        self.cs_reference = "_frame" if self.cs_reference == "_origin" else "empty"

    def base_marker(self, cs_name):
        # define marker common fields
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE
        marker.scale.x = 0.01 # Vector3(0.01, 0.01, 0)
        marker.color.g = 1.0
        marker.color.a = 1.0
        marker.header.frame_id = cs_name
        marker.pose.orientation = Quaternion(0,0,0,1)
        return marker

    def coord_sys_axes(self, cs_points):

        # read axes points
        orig = Point()  # axes origin point
        orig.x = cs_points.P[0].x * 0.001
        orig.y = cs_points.P[0].y * 0.001

        axis_x = Point() # axis x line end point 
        axis_x.x = cs_points.P[1].x * 0.001
        axis_x.y = orig.y
        
        axis_y = Point() # axis y line end point 
        axis_y.x = orig.x
        axis_y.y = cs_points.P[2].y * 0.001

        # create one marker for each axis line 
        # and append the correspondent points
        axis_x_marker = self.base_marker("[P]")
        axis_y_marker = self.base_marker("[P]")
        
        axis_x_marker.points.append(orig)
        axis_x_marker.points.append(axis_x)
        axis_y_marker.points.append(orig)
        axis_y_marker.points.append(axis_y)

        # update frame and namespace
        axis_x_marker.ns = cs_points.name + "_origin/polyline/axis_x"
        axis_y_marker.ns = cs_points.name + "_origin/polyline/axis_y"

        return axis_x_marker, axis_y_marker

    def coord_sys_frame(self, cs_points):

        frame = self.base_marker("[P]")

        # read frame points
        for i in [0,1,2,3,0]:
            point = Point()
            point.x = cs_points.P[i].x * 0.001
            point.y = cs_points.P[i].y * 0.001
            frame.points.append(point)

        frame.ns = cs_points.name + "_frame/polyline/T1_T2_T3_T4"

        return frame

    def set_coord_sys_cb(self, req):

        self.active_cs = req.name
        return CoordinateSystemNameResponse(True, "Coordinate System set as active.")
        
    def show_coord_sys_cb(self, req):

        if not self.active_cs:
            return CoordinateSystemShowResponse(False, "None Coordinate System is set.")
            
        if not req.secs > 0:
            return CoordinateSystemShowResponse(False, "Seconds projection is set to 0.")
            
        self.timer_secs = req.secs
        self.cs_reference = "_origin"
        self.update_cs_markers()
        
        return CoordinateSystemShowResponse(True, "Active Coordinate System showed correctly.")

    def remove_coord_sys_cb(self, req):

        if any(req.name in cs.ns for cs in self.cs_marker_array.markers):
            self.cs_marker_array.markers = [cs for cs in self.cs_marker_array.markers if cs.ns.find(req.name)==-1]
            self.pe_marker_array.markers = [pe for pe in self.pe_marker_array.markers if pe.ns.find(req.name)==-1]

            if req.name == self.active_cs:
                self.active_cs = ""

            return CoordinateSystemNameResponse(True, "Coordinate System removed.")
        else:
            return CoordinateSystemNameResponse(False, "Coordinate System does not exist.")

    def add_fig_cb(self, msg):

        # define marker common fields
        marker = self.base_marker(self.active_cs)

        step = self.compute_step()
        # common params from msg
        size = msg.size[0] * step
        angle = radians(msg.angle[0])
        marker.pose.position.x = msg.position.x * step 
        marker.pose.position.y = msg.position.y * step 

        if msg.figure_type == Figure.POLYLINE:
            # overwrite position for polyline ()
            marker.pose.position.x += size/2*cos(angle)
            marker.pose.position.y += size/2*sin(angle)
            figure = self.line_eq(size, angle)

        elif msg.figure_type == Figure.CIRCLE:
            figure = self.circle_eq(size, 0.0, 2*pi) # size is circle radius

        elif msg.figure_type == Figure.ARC:
            end_angle = radians(msg.angle[1]) # arc needs an end angle
            figure = self.circle_eq(size, angle, end_angle) # size is arc radius

        elif msg.figure_type == Figure.OVAL:
            height_size = msg.size[1]*0.001 # oval also needs height size
            figure = self.oval_eq(size, height_size, angle) 

        elif msg.figure_type == Figure.TEXT:
            # overwrite some marker fields for text
            marker.type = Marker.TEXT_VIEW_FACING
            rotation = Rotation.from_euler('xyz', [0, 0, angle], degrees=False)        
            marker.pose.orientation = Quaternion(*rotation.as_quat())
            marker.text = msg.text

        marker.points = figure
        marker.ns = self.active_cs+"/" + msg.projection_group + self.figures_list[msg.figure_type] + msg.figure_name
        self.pe_marker_array.markers.append(marker)

    def line_eq(self, length, ang):
        
        line_points = []
        delta_th = 0.01
        for th in np.arange(-length/2, length/2, delta_th):
            point = Point()
            point.x = th * cos(ang)
            point.y = th * sin(ang)
            line_points.append(point)

        return line_points

    def circle_eq(self, radius, start_ang, end_ang):
        
        circle_points = []
        delta_th = 0.01
        for th in np.arange(start_ang, end_ang, delta_th):
            point = Point()
            point.x = radius * sin(th)
            point.y = radius * cos(th)
            circle_points.append(point)

        return circle_points

    def oval_eq(self, a, b, angle):
        
        oval_points = []
        delta_th = 0.01
        for th in np.arange(0.0, 2*pi+delta_th, delta_th):
            point = Point()
            point.x = a * cos(th)*cos(angle) - b * sin(th)*sin(angle)
            point.y = a * cos(th)*sin(angle) + b * sin(th)*cos(angle)
            oval_points.append(point)

        return oval_points

    def hide_proj_elem_cb(self, req):

        for i, marker in enumerate(self.pe_marker_array.markers):
            if marker.ns.find(req.projection_group)>-1 and marker.ns.find(req.figure_name)>-1:
                self.pe_marker_array.markers[i].color.a = 0
                return ProjectionElementResponse(True, "Figure hidden correctly.")

        return ProjectionElementResponse(False, "Figure not found.")

    def unhide_proj_elem_cb(self, req):

        for i, marker in enumerate(self.pe_marker_array.markers):
            if marker.ns.find(req.projection_group)>-1 and marker.ns.find(req.figure_name)>-1:
                self.pe_marker_array.markers[i].color.a = 1
                return ProjectionElementResponse(True, "Figure unhidden correctly.")

        return ProjectionElementResponse(False, "Figure not found.")

    def remove_proj_elem_cb(self, req):

        for i, marker in enumerate(self.pe_marker_array.markers):
            if marker.ns.find(req.projection_group)>-1 and marker.ns.find(req.figure_name)>-1:
                self.pe_marker_array.markers.pop(i)
                return ProjectionElementResponse(True, "Figure removed correctly.")

        return ProjectionElementResponse(False, "Figure not found.")

    def translate(self, marker, dx=0, dy=0, dz=0):
        
        marker.action = Marker.DELETE
        marker.pose.position.x += dx
        marker.pose.position.y += dy
        marker.pose.position.z += dz
        marker.action = Marker.ADD

    def compute_step(self):

        res = rospy.get_param('/zlaser/coordinate_system_resolution', 1000)
        P0_x  = rospy.get_param('/zlaser/P0/x', 1000) * 0.001
        P1_x  = rospy.get_param('/zlaser/P1/x', 1000) * 0.001
        step = (P1_x - P0_x)/res

        return step

    def rotate(self, marker, angle):
        
        marker.action = Marker.DELETE
        q = marker.pose.orientation
        rotation = Rotation.from_euler('xyz', [0, 0, angle], degrees=True)
        q_rot = Quaternion(*rotation.as_quat())
        marker.pose.orientation =  self.quat_multiply(q_rot, q)
        marker.action = Marker.ADD

    def quat_multiply(self, q1, q0):

        return Quaternion( q1.x*q0.w + q1.y*q0.z - q1.z*q0.y + q1.w*q0.x,
                          -q1.x*q0.z + q1.y*q0.w + q1.z*q0.x + q1.w*q0.y,
                           q1.x*q0.y - q1.y*q0.x + q1.z*q0.w + q1.w*q0.z,
                          -q1.x*q0.x - q1.y*q0.y - q1.z*q0.z + q1.w*q0.w)

    def scale(self, marker, factor, proj_elem_params):

        marker.action = Marker.DELETE

        self.scale_factor *= factor # update factor
        
        size = proj_elem_params.size[0]*0.001 * self.scale_factor  
        angle = radians(proj_elem_params.angle[0])

        if proj_elem_params.figure_type == Figure.POLYLINE:
            figure = self.line_eq(size, angle) # size is line length

        elif proj_elem_params.figure_type == Figure.CIRCLE:
            figure = self.circle_eq(size, 0.0, 2*pi) # size is circle radius

        elif proj_elem_params.figure_type == Figure.ARC:
            end_ang = radians(proj_elem_params.angle[1])
            figure = self.circle_eq(size, angle, end_ang) # size is arc radius

        elif proj_elem_params.figure_type == Figure.OVAL:
            height_size = proj_elem_params.size[1]*0.001 * self.scale_factor 
            figure = self.oval_eq(size, height_size, angle) # size is oval width

        elif proj_elem_params.figure_type == Figure.TEXT:
            marker.scale.z = marker.scale.z*0.001 * self.scale_factor
            figure = []

        marker.points = figure
        marker.action = Marker.ADD

    def on_press(self, key, marker, proj_elem_params):
        
        if any([key in COMBO for COMBO in self.keyboard_params.COMBINATIONS]):
            self.current.add(key)

            if self.current == self.keyboard_params.KEY_UP:
                rospy.loginfo("VIZ_KEY_UP")
                self.translate(marker, dy=self.compute_step())
            elif self.current == self.keyboard_params.KEY_DOWN:
                rospy.loginfo("VIZ_KEY_DOWN")
                self.translate(marker, dy=-self.compute_step())
            elif self.current == self.keyboard_params.KEY_LEFT:
                rospy.loginfo("VIZ_KEY_LEFT")
                self.translate(marker, dx=-self.compute_step())
            elif self.current == self.keyboard_params.KEY_RIGHT:
                rospy.loginfo("VIZ_KEY_RIGHT")
                self.translate(marker, dx=self.compute_step())
            elif self.current == self.keyboard_params.KEY_PLUS:
                rospy.loginfo("VIZ_KEY_PLUS")
                self.scale(marker, 2, proj_elem_params)
            elif self.current == self.keyboard_params.KEY_MINUS:
                rospy.loginfo("VIZ_KEY_MINUS")
                self.scale(marker, 0.5, proj_elem_params)
            elif self.current == self.keyboard_params.CTRL_LEFT:
                rospy.loginfo("VIZ_CTRL_LEFT")
                self.rotate(marker, 1)
            elif self.current == self.keyboard_params.CTRL_RIGHT:
                rospy.loginfo("VIZ_CTRL_RIGHT")
                self.rotate(marker, -1)
            elif self.current == self.keyboard_params.ESC:
                rospy.loginfo("VIZ_ESC")
                marker.action = Marker.DELETE

    def on_release(self, key):
        
        if any([key in COMBO for COMBO in self.keyboard_params.COMBINATIONS]):
            if self.current == self.keyboard_params.ESC:
                return False # stop listener
            
            self.current.remove(key)

    def marker_from_name(self, name):
        for marker in self.pe_marker_array.markers:
            if name in marker.ns:
                marker.action = Marker.ADD
                return marker

    def init_keyboard_listener_cb(self, msg):

        self.keyboard_params = KeyboardParameters()
        self.current = set()
        self.scale_factor = 1
        
        name = self.active_cs + "/" + msg.projection_group + self.figures_list[msg.figure_type] + msg.figure_name

        marker = self.marker_from_name(name)

        try:
            on_press_handler = lambda event: self.on_press(event, marker=marker, proj_elem_params=msg)
            listener = keyboard.Listener(on_press = on_press_handler,
                                         on_release = self.on_release)
            listener.start()

            return ProjectionElementResponse(True, "Viz monitor.")

        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':

    rospy.init_node('zlp_viz_node')

    zlp_visualizer = ZLPVisualizer()