#! /usr/bin/env python

import rospy
import math
import numpy as np
import tf

from geometry_msgs.msg import Point, Quaternion, Vector3, Pose, Quaternion
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray
from z_laser_msgs.msg import Line, Curve, Text
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
        self.cs_element = ""

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
        
        self.add_line       = rospy.Subscriber("add_line", Line, self.add_line_cb)
        self.add_curve      = rospy.Subscriber("add_curve", Curve, self.add_curve_cb)
        self.add_text       = rospy.Subscriber("add_text", Text, self.add_text_cb)

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
        self.cs_element = "_origin"
        self.timer_secs = 3
        self.update_cs_markers()

        return CoordinateSystemResponse([], True, "Coordinate System added manually.")

    def update_cs_markers(self):

        for cs in self.cs_marker_array.markers:
            if (self.active_cs + self.cs_element) in cs.ns:
                cs.action = Marker.ADD
        
        if self.cs_element in ["_origin","_frame"]:
            rospy.Timer(rospy.Duration(self.timer_secs), self.timer_cb, oneshot=True)

        self.cs_element = "_frame" if self.cs_element == "_origin" else "empty"

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
                self.cs_element = "_origin"
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

    def add_line_cb(self,msg):
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE

        # marker.pose.position = Point(0,0,0)
        marker.pose.orientation = Quaternion(0,0,0,1)

        marker.scale.x = 0.01 # Vector3(0.01, 0.01, 0)

        marker.color.g = 1.0
        marker.color.a = 1.0

        x_start = msg.x * 0.001
        y_start = msg.y * 0.001
        length = msg.length * 0.001
        angle = msg.angle * math.pi/180

        delta_th = 0.01
        for th in np.arange(0.0, length, delta_th):
            x = x_start + th * math.cos(angle)
            y = y_start + th * math.sin(angle)

            point = Point()
            point.x = x
            point.y = y
            marker.points.append(point)

        marker.ns = self.active_cs + "/" + msg.projection_group + "/polyline/" + msg.figure_name
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def add_curve_cb(self,msg):
        if msg.curve_type == "circle":
            self.add_circle(msg)
        elif msg.curve_type == "oval":
            self.add_oval(msg)
        elif msg.curve_type == "arc":
            self.add_arc(msg)
    
    def add_circle(self,msg):
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE

        marker.pose.orientation = Quaternion(0,0,0,1)

        marker.scale.x = 0.01

        marker.color.g = 1.0
        marker.color.a = 1.0

        centre_x = msg.x * 0.001
        centre_y = msg.y * 0.001
        R = msg.length * 0.001

        delta_th = 0.01
        for th in np.arange(0.0, 2*math.pi+delta_th, delta_th):
            x = centre_x + R * math.sin(th)
            y = centre_y + R * math.cos(th)

            point = Point()
            point.x = x
            point.y = y
            marker.points.append(point)

        marker.ns = self.active_cs + "/" + msg.projection_group + "/circle/" + msg.figure_name
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def add_oval(self,msg):
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE

        marker.pose.orientation = Quaternion(0,0,0,1)

        marker.scale.x = 0.01

        marker.color.g = 1.0
        marker.color.a = 1.0

        centre_x = msg.x * 0.001
        centre_y = msg.y * 0.001
        a = msg.length * 0.001
        b = msg.height * 0.001
        angle = msg.angle * math.pi/180

        delta_th = 0.01
        for th in np.arange(0.0, 2*math.pi+delta_th, delta_th):
            x = centre_x + a * math.cos(th)*math.cos(angle) - b * math.sin(th)*math.sin(angle)
            y = centre_y + a * math.cos(th)*math.sin(angle) + b * math.sin(th)*math.cos(angle)

            point = Point()
            point.x = x
            point.y = y
            marker.points.append(point)

        marker.ns = self.active_cs + "/" + msg.projection_group + "/oval/" + msg.figure_name
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def add_arc(self,msg):
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE

        marker.pose.orientation = Quaternion(0,0,0,1)

        marker.scale.x = 0.01

        marker.color.g = 1.0
        marker.color.a = 1.0

        centre_x = msg.x * 0.001
        centre_y = msg.y * 0.001
        R = msg.length * 0.001
        start_angle = msg.angle * math.pi/180
        end_angle = msg.end_angle * math.pi/180

        delta_th = 0.01
        for th in np.arange(start_angle, end_angle, delta_th):
            x = centre_x + R * math.cos(th)
            y = centre_y + R * math.sin(th)

            point = Point()
            point.x = x
            point.y = y
            marker.points.append(point)

        marker.ns = self.active_cs + "/" + msg.projection_group + "/arc/" + msg.figure_name
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def add_text_cb(self,msg):
        marker = Marker()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.DELETE

        marker.pose.position.x = msg.x * 0.001
        marker.pose.position.y = msg.y * 0.001
        marker.pose.position.z = 0.0
        
        rotation = tf.transformations.quaternion_from_euler(0, 0, msg.angle)
        marker.pose.orientation = Quaternion(rotation[0],rotation[1],rotation[2],rotation[3])

        marker.scale.z = msg.height * 0.001

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

if __name__ == '__main__':

    rospy.init_node('zlp_viz_node')

    zlp_visualizer = ZLPVisualizer()