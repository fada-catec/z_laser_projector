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

class ZLPSimulator(object):

    def __init__(self):

        self.cs_marker_array = MarkerArray()
        self.pe_marker_array = MarkerArray()

        self.active_cs = ""
        self.coordinate_system_list = []

        # self.stop = True

    def open_services(self):
        
        self.start_proj    = rospy.Service('projection_start', Trigger, self.projection_start_cb)
        self.stop_proj     = rospy.Service('projection_stop', Trigger, self.projection_stop_cb)

        self.manual_cs     = rospy.Service('define_coordinate_system', CoordinateSystem, self.manual_define_coord_sys_cb)

        self.set_cs        = rospy.Service('set_coordinate_system', CoordinateSystemName, self.set_coord_sys_cb)
        self.rem_cs        = rospy.Service('remove_coordinate_system', CoordinateSystemName, self.remove_coord_sys_cb)
        self.show_cs       = rospy.Service('show_active_coordinate_system', CoordinateSystemShow, self.show_coord_sys_cb)

        self.hide_shape    = rospy.Service('hide_shape', ProjectionElement, self.hide_shape_cb)
        self.unhide_shape  = rospy.Service('unhide_shape', ProjectionElement, self.unhide_shape_cb)
        self.remove_shape  = rospy.Service('remove_shape', ProjectionElement, self.remove_shape_cb)
        
        self.add_line      = rospy.Subscriber("add_line", Line, self.add_line_cb)
        self.add_curve     = rospy.Subscriber("add_curve", Curve, self.add_curve_cb)
        self.add_text      = rospy.Subscriber("add_text", Text, self.add_text_cb)

        self.cs_markers_pub = rospy.Publisher('coord_sys_markers', MarkerArray, queue_size=10)
        self.pe_markers_pub = rospy.Publisher('proj_elem_markers', MarkerArray, queue_size=10)

        rospy.loginfo("Use ROS Services: \n\t\t\trosservice list")

    def projection_start_cb(self,req):
        rospy.loginfo("Received request to start projection")

        # self.stop = False

        if self.active_cs:
            for i, element in enumerate(self.pe_marker_array.markers):
                if element.ns.find(self.active_cs)>-1 and element.id == 1:
                    self.pe_marker_array.markers[i].action = Marker.ADD
        else:
            return TriggerResponse(False, "No Coordinate System set as active.")
        
        try:
            self.pe_markers_pub.publish(self.pe_marker_array)
            
        except Exception as e:
            rospy.logwarn(e)
            
        return TriggerResponse(True, "Projection started.")
    
    def projection_stop_cb(self,req):
        rospy.loginfo("Received request to stop projection.")

        # self.stop = True

        for i in range(len(self.pe_marker_array.markers)):
            self.pe_marker_array.markers[i].action = Marker.DELETE

        try:
            self.pe_markers_pub.publish(self.pe_marker_array)

        except Exception as e:
            rospy.logwarn(e)

        return TriggerResponse(True, "Projection stopped. Press RESET at rviz.")

    def manual_define_coord_sys_cb(self,req):
        rospy.loginfo("Received request to create a new coordinate system manually.")

        rospy.set_param('coordinate_system_name', req.name)
        rospy.set_param('coordinate_system_distance', req.distance)
        rospy.set_param('P1/x', req.P1.x)
        rospy.set_param('P1/y', req.P1.y)
        # rospy.set_param('T1/x', req.T1.x)
        # rospy.set_param('T1/y', req.T1.y)

        self.active_cs = req.name

        self.coord_sys_axes(req)
        self.cs_markers_pub.publish(self.cs_marker_array)
        rospy.sleep(3)

        for i in range(len(self.cs_marker_array.markers)):
            self.cs_marker_array.markers[i].action = Marker.DELETE
        
        self.coord_sys_frame(req)
        self.cs_markers_pub.publish(self.cs_marker_array)
        rospy.sleep(3)

        for i in range(len(self.cs_marker_array.markers)):
            self.cs_marker_array.markers[i].action = Marker.DELETE

        self.cs_markers_pub.publish(self.cs_marker_array)

        self.coordinate_system_list.append(req)

        return CoordinateSystemResponse([], True, "Coordinate System added manually.")

    def coord_sys_axes(self,req):

        axis_x = Marker()
        axis_x.type = Marker.LINE_STRIP
        axis_x.action = Marker.ADD

        # axis_x.pose.position = Point(0,0,0)
        axis_x.pose.orientation = Quaternion(0,0,0,1)

        # axis_x.scale = Vector3(0.01, 0.01, 0)
        axis_x.scale.x = 0.01

        axis_x.color.g = 1.0
        axis_x.color.a = 1.0

        point = Point()
        point.x = req.P1.x * 0.001
        point.y = req.P1.y * 0.001
        axis_x.points.append(point)
        point = Point()
        point.x = (req.P1.x + (req.P2.x - req.P1.x)/3) * 0.001
        point.y = req.P1.y * 0.001
        axis_x.points.append(point)

        axis_x.ns = req.name + "_origin/" + "axis_x"
        axis_x.header.frame_id = "[P]"

        self.cs_marker_array.markers.append(axis_x)

        axis_y = Marker()
        axis_y.type = Marker.LINE_STRIP
        axis_y.action = Marker.ADD

        # axis_y.pose.position = Point(0,0,0)
        axis_y.pose.orientation = Quaternion(0,0,0,1)

        # axis_y.scale = Vector3(0.01, 0.01, 0)
        axis_y.scale.x = 0.01

        axis_y.color.g = 1.0
        axis_y.color.a = 1.0

        point = Point()
        point.x = req.P1.x * 0.001
        point.y = req.P1.y * 0.001
        axis_y.points.append(point)
        point = Point()
        point.x = req.P1.x * 0.001
        point.y = (req.P1.y + (req.P3.y - req.P1.y)/3) * 0.001
        axis_y.points.append(point)

        axis_y.ns = req.name + "_origin/" + "axis_y"
        axis_y.header.frame_id = "[P]"

        self.cs_marker_array.markers.append(axis_y)

    def coord_sys_frame(self,req):
        # Se podria usar la funcion add_line en vez de repetir esto pero 
        # haciendolo de esta forma se crea un unico marker con forma de frame
        # en vez de 4 lines que conforman el frame
        frame = Marker()
        frame.type = Marker.LINE_STRIP
        frame.action = Marker.ADD

        # frame.pose.position = Point(0,0,0)
        frame.pose.orientation = Quaternion(0,0,0,1)

        # frame.scale = Vector3(0.01, 0.01, 0)
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

        frame.ns = req.name + "_frame/" + "T1_T2_T3_T4"
        frame.header.frame_id = "[P]"

        self.cs_marker_array.markers.append(frame)

    def set_coord_sys_cb(self,req):
        rospy.loginfo("Received request to set coordinate system.")

        cs_found = None
        for cs in self.coordinate_system_list:
            if cs.name.find(req.name) > -1:
                cs_found = cs
                break

        if cs_found:
            self.active_cs = cs_found.name
            rospy.set_param('coordinate_system_name', cs_found.name)
            rospy.set_param('coordinate_system_distance', cs_found.distance)
            rospy.set_param('P1/x', cs_found.P1.x)
            rospy.set_param('P1/y', cs_found.P1.y)
            return CoordinateSystemNameResponse(True, "Coordinate System set as active.")
        else:
            return CoordinateSystemNameResponse(False, "Coordinate System does not exist.")
        
    def show_coord_sys_cb(self,req):
        rospy.loginfo("Request to project active coordinate system.")

        if self.active_cs:

            if req.secs > 0:

                for i, element in enumerate(self.cs_marker_array.markers):
                    if element.ns.find(self.active_cs) > -1 and element.ns.find("origin") > -1:
                        self.cs_marker_array.markers[i].action = Marker.ADD
                self.cs_markers_pub.publish(self.cs_marker_array)
                rospy.sleep(req.secs)
                
                for i, element in enumerate(self.cs_marker_array.markers):
                    if element.ns.find(self.active_cs) > -1 and element.ns.find("frame") > -1:
                        self.cs_marker_array.markers[i].action = Marker.ADD
                    else:
                        self.cs_marker_array.markers[i].action = Marker.DELETE
                self.cs_markers_pub.publish(self.cs_marker_array)
                rospy.sleep(req.secs)

                for i in range(len(self.cs_marker_array.markers)):
                    self.cs_marker_array.markers[i].action = Marker.DELETE
                self.cs_markers_pub.publish(self.cs_marker_array)

                return CoordinateSystemShowResponse(True, "Active Coordinate System showed correctly.")
            else:
                return CoordinateSystemShowResponse(False, "Seconds projection is set to 0.")

        else:
            return CoordinateSystemShowResponse(False, "None Coordinate System is set.")

    def remove_coord_sys_cb(self,req):
        rospy.loginfo("Received request to remove coordinate system")
 
        index_cs = [i for i, element in enumerate(self.cs_marker_array.markers) if element.ns.find(req.name)>-1]
        if index_cs:
            self.cs_marker_array.markers = [v for i, v in enumerate(self.cs_marker_array.markers) if i not in index_cs]
            self.cs_markers_pub.publish(self.cs_marker_array)

            index_pe = [i for i, element in enumerate(self.pe_marker_array.markers) if element.ns.find(req.name)>-1]
            if index_pe:
                self.pe_marker_array.markers = [v for i, v in enumerate(self.pe_marker_array.markers) if i not in index_pe]
                self.pe_markers_pub.publish(self.pe_marker_array)

            if req.name == self.active_cs:
                self.active_cs = ""
                rospy.set_param('coordinate_system_name', "")
                rospy.set_param('coordinate_system_distance', 1500)
                rospy.set_param('P1/x', 0)
                rospy.set_param('P1/y', 0)
                rospy.logwarn("Set or define new coordinate system.")

            i = 0
            for cs in self.coordinate_system_list:
                if cs.name.find(req.name)>-1:
                    self.coordinate_system_list.pop(i)
                    break
                i += 1

            return CoordinateSystemNameResponse(True, "Coordinate System removed.")
        
        else:
            return CoordinateSystemNameResponse(False, "Coordinate System does not exist.")

    def add_line_cb(self,msg):
        rospy.loginfo("Received request to add a line to the active coordinate system.")

        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE # por defecto DELETE
                                      #no elimina el marker, lo oculta del rviz

        # marker.pose.position = Point(0,0,0)
        marker.pose.orientation = Quaternion(0,0,0,1)

        # marker.scale = Vector3(0.01, 0.01, 0)
        marker.scale.x = 0.01

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

        # El campo id del msg Marker lo uso para identificar si estan hidden(0) o unhidden(1)
        # Cuando se crea una figura (linea, circulo, etc) se pone inicialmente a 1
        marker.id = 1
        marker.ns = self.active_cs + "/" + msg.group_name + "/polyline/" + msg.shape_id
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def add_curve_cb(self,msg):

        if msg.shape_type == "circle":
            rospy.loginfo("Received request to add a circle to the active coordinate system.")
            self.add_circle(msg)
        elif msg.shape_type == "oval":
            rospy.loginfo("Received request to add an oval to the active coordinate system.")
            self.add_oval(msg)
        elif msg.shape_type == "arc":
            rospy.loginfo("Received request to add an arc to the active coordinate system.")
            self.add_arc(msg)
    
    def add_circle(self,msg):

        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE

        # marker.pose.position = Point(0,0,0)
        marker.pose.orientation = Quaternion(0,0,0,1)

        # marker.scale = Vector3(0.01, 0.01, 0)
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

        marker.id = 1
        marker.ns = self.active_cs + "/" + msg.group_name + "/circle/" + msg.shape_id
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def add_oval(self,msg):

        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE

        # marker.pose.position = Point(0,0,0)
        marker.pose.orientation = Quaternion(0,0,0,1)

        # marker.scale = Vector3(0.01, 0.01, 0)
        marker.scale.x = 0.01

        marker.color.g = 1.0
        marker.color.a = 1.0

        centre_x = msg.x * 0.001
        centre_y = msg.y * 0.001
        a = msg.length * 0.001
        b = msg.height * 0.001

        delta_th = 0.01
        for th in np.arange(0.0, 2*math.pi+delta_th, delta_th):
            x = centre_x + a * math.sin(th)
            y = centre_y + b * math.cos(th)

            point = Point()
            point.x = x
            point.y = y

            marker.points.append(point)

        marker.id = 1
        marker.ns = self.active_cs + "/" + msg.group_name + "/oval/" + msg.shape_id
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def add_arc(self,msg):

        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE

        # marker.pose.position = Point(0,0,0)
        marker.pose.orientation = Quaternion(0,0,0,1)

        # marker.scale = Vector3(0.01, 0.01, 0)
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
            x = centre_x + R * math.sin(th)
            y = centre_y + R * math.cos(th)

            point = Point()
            point.x = x
            point.y = y

            marker.points.append(point)

        marker.id = 1
        marker.ns = self.active_cs + "/" + msg.group_name + "/arc/" + msg.shape_id
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def add_text_cb(self,msg):

        marker = Marker()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.DELETE

        # marker.pose.position = Point(0,0,0)
        marker.pose.position.x = msg.x * 0.001
        marker.pose.position.y = msg.y * 0.001
        marker.pose.position.z = 0.0
        # marker.pose.orientation = Quaternion(0,0,0,1)
        rotation = tf.transformations.quaternion_from_euler(0, 0, msg.angle)
        marker.pose.orientation = Quaternion(rotation[0],rotation[1],rotation[2],rotation[3])

        # marker.scale = Vector3(0.01, 0.01, 0)
        marker.scale.z = msg.height * 0.001

        marker.text = msg.text

        marker.color.g = 1.0
        marker.color.a = 1.0

        marker.id = 1
        marker.ns = self.active_cs + "/" + msg.group_name + "/text/" + msg.shape_id
        marker.header.frame_id = self.active_cs

        self.pe_marker_array.markers.append(marker)

    def hide_shape_cb(self,req):
        rospy.loginfo("Received request to hide figure.")

        for i, element in enumerate(self.pe_marker_array.markers):
            if element.ns.find(req.group_name)>-1 and element.ns.find(req.shape_id)>-1:
                self.pe_marker_array.markers[i].id = 0
                return ProjectionElementResponse(True, "Figure hidden correctly.")

        return ProjectionElementResponse(False, "Figure not found.")

    def unhide_shape_cb(self,req):
        rospy.loginfo("Received request to unhide figure.")

        for i, element in enumerate(self.pe_marker_array.markers):
            if element.ns.find(req.group_name)>-1 and element.ns.find(req.shape_id)>-1:
                self.pe_marker_array.markers[i].id = 1
                return ProjectionElementResponse(True, "Figure unhidden correctly.")

        return ProjectionElementResponse(False, "Figure not found.")

    def remove_shape_cb(self,req):
        rospy.loginfo("Received request to remove figure.")

        for i, element in enumerate(self.pe_marker_array.markers):
            if element.ns.find(req.group_name)>-1 and element.ns.find(req.shape_id)>-1:
                self.pe_marker_array.markers.pop(i)
                return ProjectionElementResponse(True, "Figure removed correctly.")

        return ProjectionElementResponse(False, "Figure not found.")






    # def publish_loop(self):

        # self.stop = True

        # while True:

        #     if not self.stop:
        #         self.pe_markers_pub.publish(self.pe_marker_array)

        #     rospy.Rate(10)






if __name__ == '__main__':

    rospy.init_node('zlp_viz_node')

    zlp_simulator = ZLPSimulator()

    zlp_simulator.open_services()

    # zlp_simulator.publish_loop()

    rospy.spin()
         