#! /usr/bin/env python3

import rospy
import math
from math import sin,cos,pi
import numpy as np
from scipy.spatial.transform import Rotation
from pynput import keyboard

from z_laser_zlp1.zlp_utils import KeyboardParameters
from z_laser_zlp1.zlp_utils import CoordinateSystemParameters, ProjectionElementParameters

from geometry_msgs.msg import Point, Quaternion, Vector3, Pose, Quaternion
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray
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
        self.STD_WAIT_TIME = CoordinateSystemParameters().DEFAULT_WAIT_TIME

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

        self.hide_figure    = rospy.Service('hide_figure', ProjectionElement, self.hide_figure_cb)
        self.unhide_figure  = rospy.Service('unhide_figure', ProjectionElement, self.unhide_figure_cb)
        self.remove_figure  = rospy.Service('remove_figure', ProjectionElement, self.remove_figure_cb)
        
        self.add_proj_elem  = rospy.Subscriber("add_projection_element", Figure, self.add_fig_cb)
        self.monit_figure   = rospy.Subscriber("monitor_figure", Figure, self.init_keyboard_listener_cb)

    def projection_start_cb(self,req):

        if self.active_cs:
            for i, element in enumerate(self.pe_marker_array.markers):
                if element.ns.find(self.active_cs)>-1:
                    self.pe_marker_array.markers[i].action = Marker.ADD
        else:
            return TriggerResponse(False, "No Coordinate System set as active.")
        
        return TriggerResponse(True, "Projection started.")
    
    def projection_stop_cb(self,req):

        for i in range(len(self.pe_marker_array.markers)):
            self.pe_marker_array.markers[i].action = Marker.DELETE

        return TriggerResponse(True, "Projection stopped.")

    def manual_define_coord_sys_cb(self,req):
        
        for element in self.cs_marker_array.markers:
            if req.name in element.ns:
                return CoordinateSystemResponse([], False, "Coordinate System already exists.")

        self.active_cs = req.name

        axis_x, axis_y = self.coord_sys_axes(req)
        self.cs_marker_array.markers.append(axis_x)
        self.cs_marker_array.markers.append(axis_y)
        frame = self.coord_sys_frame(req)
        self.cs_marker_array.markers.append(frame)

        self.active_cs = req.name
        self.cs_reference = "_origin"
        self.timer_secs = self.STD_WAIT_TIME
        self.update_cs_markers()

        return CoordinateSystemResponse([], True, "Coordinate System added manually.")

    def update_cs_markers(self):

        for cs in self.cs_marker_array.markers:
            if (self.active_cs + self.cs_reference) in cs.ns:
                cs.action = Marker.ADD
        
        if self.cs_reference in ["_origin","_frame"]:
            rospy.Timer(rospy.Duration(self.timer_secs), self.timer_cb, oneshot=True)

        self.cs_reference = "_frame" if self.cs_reference == "_origin" else "empty"

    def timer_cb(self, timer):

        for i in range(len(self.cs_marker_array.markers)):
            self.cs_marker_array.markers[i].action = Marker.DELETE

        self.update_cs_markers()

    def coord_sys_axes(self,req):
        axis_x = Marker()
        axis_x.type = Marker.LINE_STRIP
        axis_x.action = Marker.DELETE

        axis_x.pose.orientation = Quaternion(0,0,0,1)

        axis_x.scale.x = 0.01

        axis_x.color.g = 1.0
        axis_x.color.a = 1.0

        point = Point()
        point.x = req.P1.x * 0.001
        point.y = req.P1.y * 0.001
        axis_x.points.append(point)
        point = Point()
        point.x = (req.P1.x + (req.P2.x - req.P1.x)) * 0.001
        point.y = req.P1.y * 0.001
        axis_x.points.append(point)

        axis_x.ns = req.name + "_origin/polyline/axis_x"
        axis_x.header.frame_id = "[P]"

        axis_y = Marker()
        axis_y.type = Marker.LINE_STRIP
        axis_y.action = Marker.DELETE

        axis_y.pose.orientation = Quaternion(0,0,0,1)

        axis_y.scale.x = 0.01

        axis_y.color.g = 1.0
        axis_y.color.a = 1.0

        point = Point()
        point.x = req.P1.x * 0.001
        point.y = req.P1.y * 0.001
        axis_y.points.append(point)
        point = Point()
        point.x = req.P1.x * 0.001
        point.y = (req.P1.y + (req.P3.y - req.P1.y)) * 0.001
        axis_y.points.append(point)

        axis_y.ns = req.name + "_origin/polyline/axis_y"
        axis_y.header.frame_id = "[P]"

        return axis_x,axis_y

    def coord_sys_frame(self,req):

        frame = Marker()
        frame.type = Marker.LINE_STRIP
        frame.action = Marker.DELETE

        frame.pose.orientation = Quaternion(0,0,0,1)
        frame.scale.x = 0.01

        frame.color.g = 1.0
        frame.color.a = 1.0

        point = Point()
        point.x = req.P1.x * 0.001
        point.y = req.P1.y * 0.001
        frame.points.append(point)
        point = Point()
        point.x = req.P2.x * 0.001
        point.y = req.P2.y * 0.001
        frame.points.append(point)
        point = Point()
        point.x = req.P3.x * 0.001
        point.y = req.P3.y * 0.001
        frame.points.append(point)
        point = Point()
        point.x = req.P4.x * 0.001
        point.y = req.P4.y * 0.001
        frame.points.append(point)
        point = Point()
        point.x = req.P1.x * 0.001
        point.y = req.P1.y * 0.001
        frame.points.append(point)

        frame.ns = req.name + "_frame/polyline/T1_T2_T3_T4"
        frame.header.frame_id = "[P]"

        return frame

    def set_coord_sys_cb(self,req):

        self.active_cs = req.name
        return CoordinateSystemNameResponse(True, "Coordinate System set as active.")
        
    def show_coord_sys_cb(self,req):

        if self.active_cs:

            if req.secs > 0:

                self.timer_secs = req.secs
                self.cs_reference = "_origin"
                self.update_cs_markers()
                return CoordinateSystemShowResponse(True, "Active Coordinate System showed correctly.")
            else:
                return CoordinateSystemShowResponse(False, "Seconds projection is set to 0.")
        else:
            return CoordinateSystemShowResponse(False, "None Coordinate System is set.")

    def remove_coord_sys_cb(self,req):

        if any(req.name in cs.ns for cs in self.cs_marker_array.markers):
            self.cs_marker_array.markers = [cs for cs in self.cs_marker_array.markers if cs.ns.find(req.name)==-1]
            self.pe_marker_array.markers = [pe for pe in self.pe_marker_array.markers if pe.ns.find(req.name)==-1]

            if req.name == self.active_cs:
                self.active_cs = ""

            return CoordinateSystemNameResponse(True, "Coordinate System removed.")
        else:
            return CoordinateSystemNameResponse(False, "Coordinate System does not exist.")

    def add_fig_cb(self,msg):

        if msg.figure_type == 0:
            self.add_line(msg)
        elif msg.figure_type == 1:
            self.add_circle(msg)
        elif msg.figure_type == 2:
            self.add_arc(msg)
        elif msg.figure_type == 3:
            self.add_oval(msg)
        elif msg.figure_type == 4:
            self.add_text(msg)

    def add_line(self,msg):

        x_start = msg.position.x * 0.001
        y_start = msg.position.y * 0.001
        length = msg.size[0] * 0.001
        angle = msg.angle[0] * math.pi/180

        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE

        marker.pose.position = Point(x_start + length/2*math.cos(angle),
                                    y_start + length/2*math.sin(angle),
                                    0)
        marker.pose.orientation = Quaternion(0,0,0,1)

        marker.scale.x = 0.01 # Vector3(0.01, 0.01, 0)

        marker.color.g = 1.0
        marker.color.a = 1.0

        figure = self.line_eq(length,angle)
        marker.points = figure

        marker.ns = self.active_cs + "/" + msg.projection_group + "/polyline/" + msg.figure_name
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def line_eq(self,leng,ang):
        
        traject = []

        delta_th = 0.01
        for th in np.arange(-leng/2, leng/2, delta_th):
            x = th * math.cos(ang)
            y = th * math.sin(ang)

            point = Point()
            point.x = x
            point.y = y
            traject.append(point)

        return traject
    
    def add_circle(self,msg):

        centre_x = msg.position.x * 0.001
        centre_y = msg.position.y * 0.001
        radius = msg.size[0] * 0.001

        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE

        marker.pose.position = Point(centre_x,centre_y,0)
        marker.pose.orientation = Quaternion(0,0,0,1)

        marker.scale.x = 0.01

        marker.color.g = 1.0
        marker.color.a = 1.0

        figure = self.circle_eq(radius,0.0,2*math.pi)
        marker.points = figure

        marker.ns = self.active_cs + "/" + msg.projection_group + "/circle/" + msg.figure_name
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def add_arc(self,msg):

        centre_x = msg.position.x * 0.001
        centre_y = msg.position.y * 0.001
        radius = msg.size[0] * 0.001
        start_angle = msg.angle[0] * math.pi/180
        end_angle = msg.angle[1] * math.pi/180

        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE

        marker.pose.position = Point(centre_x,centre_y,0)
        marker.pose.orientation = Quaternion(0,0,0,1)

        marker.scale.x = 0.01

        marker.color.g = 1.0
        marker.color.a = 1.0

        figure = self.circle_eq(radius,start_angle,end_angle)
        marker.points = figure

        marker.ns = self.active_cs + "/" + msg.projection_group + "/arc/" + msg.figure_name
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def circle_eq(self,radius,start_ang,end_ang):
        
        traject = []

        delta_th = 0.01
        # for th in np.arange(0.0, 2*math.pi+delta_th, delta_th):
        for th in np.arange(start_ang, end_ang, delta_th):
            x = radius * math.sin(th)
            y = radius * math.cos(th)

            point = Point()
            point.x = x
            point.y = y
            traject.append(point)

        return traject

    def add_oval(self,msg):

        centre_x = msg.position.x * 0.001
        centre_y = msg.position.y * 0.001
        width = msg.size[0] * 0.001
        height = msg.size[1] * 0.001
        angle = msg.angle[0] * math.pi/180

        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE

        marker.pose.position = Point(centre_x,centre_y,0)
        marker.pose.orientation = Quaternion(0,0,0,1)

        marker.scale.x = 0.01

        marker.color.g = 1.0
        marker.color.a = 1.0

        figure = self.oval_eq(width,height,angle)
        marker.points = figure

        marker.ns = self.active_cs + "/" + msg.projection_group + "/oval/" + msg.figure_name
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def oval_eq(self,a,b,angle):
        
        traject = []

        delta_th = 0.01
        for th in np.arange(0.0, 2*math.pi+delta_th, delta_th):
            x = a * math.cos(th)*math.cos(angle) - b * math.sin(th)*math.sin(angle)
            y = a * math.cos(th)*math.sin(angle) + b * math.sin(th)*math.cos(angle)

            point = Point()
            point.x = x
            point.y = y
            traject.append(point)

        return traject

    def add_text(self,msg):
        marker = Marker()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.DELETE

        marker.pose.position.x = msg.position.x * 0.001
        marker.pose.position.y = msg.position.y * 0.001
        marker.pose.position.z = 0.0
        
        rotation = Rotation.from_euler('xyz', [0, 0, msg.angle[0]], degrees=True)        
        marker.pose.orientation = Quaternion(rotation.as_quat()[0],
                                            rotation.as_quat()[1],
                                            rotation.as_quat()[2],
                                            rotation.as_quat()[3])

        marker.scale.z = msg.size[0] * 0.001

        marker.text = msg.text

        marker.color.g = 1.0
        marker.color.a = 1.0

        marker.ns = self.active_cs + "/" + msg.projection_group + "/text/" + msg.figure_name
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def hide_figure_cb(self,req):

        for i, element in enumerate(self.pe_marker_array.markers):
            if element.ns.find(req.projection_group)>-1 and element.ns.find(req.figure_name)>-1:
                self.pe_marker_array.markers[i].color.a = 0
                return ProjectionElementResponse(True, "Figure hidden correctly.")

        return ProjectionElementResponse(False, "Figure not found.")

    def unhide_figure_cb(self,req):

        for i, element in enumerate(self.pe_marker_array.markers):
            if element.ns.find(req.projection_group)>-1 and element.ns.find(req.figure_name)>-1:
                self.pe_marker_array.markers[i].color.a = 1
                return ProjectionElementResponse(True, "Figure unhidden correctly.")

        return ProjectionElementResponse(False, "Figure not found.")

    def remove_figure_cb(self,req):

        for i, element in enumerate(self.pe_marker_array.markers):
            if element.ns.find(req.projection_group)>-1 and element.ns.find(req.figure_name)>-1:
                self.pe_marker_array.markers.pop(i)
                return ProjectionElementResponse(True, "Figure removed correctly.")

        return ProjectionElementResponse(False, "Figure not found.")

    def translate(self,elem,dx,dy,dz):
        
        elem.action = Marker.DELETE
        offset = Point(dx,dy,dz)
        elem.pose.position.x = elem.pose.position.x + offset.x
        elem.pose.position.y = elem.pose.position.y + offset.y
        elem.pose.position.z = elem.pose.position.z + offset.z
        elem.action = Marker.ADD

    def rotate(self,elem,angle):
        
        elem.action = Marker.DELETE
        orig = [elem.pose.orientation.x,
                elem.pose.orientation.y,
                elem.pose.orientation.z,
                elem.pose.orientation.w]
        rotation = Rotation.from_euler('xyz', [0, 0, angle], degrees=True)
        result = self.quat_multiply(rotation.as_quat(),orig)
        elem.pose.orientation = Quaternion(result[0],result[1],result[2],result[3])
        elem.action = Marker.ADD

    def quat_multiply(self,q1,q0):

        x0, y0, z0, w0 = q0
        x1, y1, z1, w1 = q1
        return np.array((x1*w0 + y1*z0 - z1*y0 + w1*x0,
                        -x1*z0 + y1*w0 + z1*x0 + w1*y0,
                        x1*y0 - y1*x0 + z1*w0 + w1*z0,
                        -x1*x0 - y1*y0 - z1*z0 + w1*w0), dtype=np.float64)

    def scale(self,elem,factor,proj_elem_params):

        elem.action = Marker.DELETE

        self.scale_factor = self.scale_factor * factor
        if proj_elem_params.figure_type == 0:
            length = proj_elem_params.size[0] * self.scale_factor  * 0.001
            angle = proj_elem_params.angle[0] * math.pi/180
            figure = self.line_eq(length,angle)
            elem.points = []
            elem.points = figure
        elif proj_elem_params.figure_type == 1:
            radius = proj_elem_params.size[0] * self.scale_factor * 0.001
            figure = self.circle_eq(radius,0.0,2*math.pi)
            elem.points = []
            elem.points = figure
        elif proj_elem_params.figure_type == 2:
            radius = proj_elem_params.size[0] * self.scale_factor * 0.001
            start_ang = proj_elem_params.angle[0] * math.pi/180
            end_ang = proj_elem_params.angle[1] * math.pi/180
            figure = self.circle_eq(radius,start_ang,end_ang)
            elem.points = []
            elem.points = figure
        elif proj_elem_params.figure_type == 3:
            width = proj_elem_params.size[0] * self.scale_factor * 0.001
            height = proj_elem_params.size[1] * self.scale_factor * 0.001
            angle = proj_elem_params.angle[0] * math.pi/180
            figure = self.oval_eq(width,height,angle)
            elem.points = []
            elem.points = figure
        elif proj_elem_params.figure_type == 4:
            elem.scale.z = elem.scale.z * self.scale_factor * 0.001
        
        elem.action = Marker.ADD

    def on_press(self,key,elem,proj_elem_params):
        
        if any([key in COMBO for COMBO in self.keyboard_params.COMBINATIONS]):
            self.current.add(key)

            if self.current == self.keyboard_params.KEY_UP:
                print ("KEY_UP")
                self.translate(elem,0,0.001,0)
            elif self.current == self.keyboard_params.KEY_DOWN:
                print ("KEY_DOWN")
                self.translate(elem,0,-0.001,0)
            elif self.current == self.keyboard_params.KEY_LEFT:
                print ("KEY_LEFT")
                self.translate(elem,-0.001,0,0)
            elif self.current == self.keyboard_params.KEY_RIGHT:
                print ("KEY_RIGHT")
                self.translate(elem,0.001,0,0)
            elif self.current == self.keyboard_params.KEY_PLUS:
                print ("KEY_PLUS")
                self.scale(elem,2,proj_elem_params)
            elif self.current == self.keyboard_params.KEY_MINUS:
                print ("KEY_MINUS")
                self.scale(elem,0.5,proj_elem_params)
            elif self.current == self.keyboard_params.COMB_1:
                print ("COMB_1")
                self.rotate(elem,1)
            elif self.current == self.keyboard_params.COMB_2:
                print ("COMB_2")
                self.rotate(elem,-1)
            elif self.current == self.keyboard_params.ESC:
                print ("ESC")
                elem.action = Marker.DELETE

    def on_release(self,key):
        
        if any([key in COMBO for COMBO in self.keyboard_params.COMBINATIONS]):
            if self.current == self.keyboard_params.ESC:
                self.current.remove(key)
                return False
            
            self.current.remove(key)

    def init_keyboard_listener_cb(self,msg):

        self.keyboard_params = KeyboardParameters()

        self.current = set()

        self.scale_factor = 1

        name = self.active_cs + "/" + msg.projection_group + self.figures_list[msg.figure_type] + msg.figure_name
        # marker = [element for element in self.pe_marker_array.markers if name in element.ns]
        for element in self.pe_marker_array.markers:
            if name in element.ns:
                marker = element
                marker.action = Marker.ADD
                break

        try:
            listener = keyboard.Listener(on_press=lambda event: self.on_press(event, elem=marker, proj_elem_params=msg),
                                        on_release=self.on_release)
            listener.start()

            return ProjectionElementResponse(True, "Viz monitor.")

        except Exception as e:
            print(e)

if __name__ == '__main__':

    rospy.init_node('zlp_viz_node')

    zlp_visualizer = ZLPVisualizer()