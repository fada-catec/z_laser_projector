# Copyright (c) 2020, FADA-CATEC

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This module contains utility classes and methods to run the ZLP GUI."""

import sys
import os
import rospy
import rospkg

from PyQt5 import QtCore, QtGui, QtWidgets, uic

from z_laser_gui.main_window import Ui_MainWindow

from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Point
from z_laser_msgs.msg import Figure
from z_laser_msgs.srv import CoordinateSystem, CoordinateSystemRequest, CoordinateSystemResponse
from z_laser_msgs.srv import CoordinateSystemName, CoordinateSystemNameRequest, CoordinateSystemNameResponse
from z_laser_msgs.srv import CoordinateSystemShow, CoordinateSystemShowRequest, CoordinateSystemShowResponse
from z_laser_msgs.srv import CoordinateSystemList, CoordinateSystemListResponse
from z_laser_msgs.srv import ProjectionElement, ProjectionElementRequest, ProjectionElementResponse


class MyWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    """This class implement the functions related with the projector Qt GUI.
    
    Attributes:
        ON_LED (str): path file of icon image ON
        OFF_LED (str): path file of icon image OFF
        projector_connected (bool): projector connection status
    """
    def __init__(self):
        """Initialize MyWindow object."""
        QtWidgets.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('z_laser_gui')
        self.ON_LED  = pkg_path + "/images/icons/ON.png"
        self.OFF_LED = pkg_path + "/images/icons/OFF.png"

        self.projector_connected = False
        self.set_up_gui()

    def set_up_gui(self):
        """Initial GUI set up."""
        # LEDS
        self.status.setPixmap(QtGui.QPixmap(self.OFF_LED))
        self.status_2.setPixmap(QtGui.QPixmap(self.OFF_LED))

        # Msgs box
        self.msg_box = QtWidgets.QMessageBox()

        # Set ComboBox callback
        self.pe_menu.currentIndexChanged.connect(self.update_entries) ###

        self.init_pe_menu()

        self.set_buttons()

        self.open_services()

        rospy.wait_for_service("connect")
        self.projector_connected = rospy.get_param('projector_connected')

        if self.projector_connected:
            self.status.setPixmap(QtGui.QPixmap(self.ON_LED))
        else:
            self.status.setPixmap(QtGui.QPixmap(self.OFF_LED))

    def init_pe_menu(self):
        """Hide most of input text boxes from projection element GUI submenu."""
        # Projection element textbox entries initial status
        self.length.setEnabled(False)
        self.length.setVisible(False)
        self.pe_length.setEnabled(False)
        self.pe_length.setVisible(False)
        self.mm_len.setEnabled(False)
        self.mm_len.setVisible(False)
        self.start_ang.setEnabled(False)
        self.start_ang.setVisible(False)
        self.pe_angle.setEnabled(False)
        self.pe_angle.setVisible(False)
        self.deg_start.setEnabled(False)
        self.deg_start.setVisible(False)
        self.end_ang.setEnabled(False)
        self.end_ang.setVisible(False)
        self.pe_end_angle.setEnabled(False)
        self.pe_end_angle.setVisible(False)
        self.deg_end.setEnabled(False)
        self.deg_end.setVisible(False)
        self.height.setEnabled(False)
        self.height.setVisible(False)
        self.pe_height.setEnabled(False)
        self.pe_height.setVisible(False)
        self.mm_height.setEnabled(False)
        self.mm_height.setVisible(False)
        self.text.setEnabled(False)
        self.text.setVisible(False)
        self.pe_text.setEnabled(False)
        self.pe_text.setVisible(False)
        self.char_spac.setEnabled(False)
        self.char_spac.setVisible(False)
        self.pe_char_space.setEnabled(False)
        self.pe_char_space.setVisible(False)

    def set_buttons(self):
        """Set callbacks of GUI buttons."""
        self.connect_button.clicked.connect(self.connect_cb)
        self.disconnect_button.clicked.connect(self.disconnect_cb)
        self.start_button.clicked.connect(self.start_proj_cb)
        self.stop_button.clicked.connect(self.stop_proj_cb)
        self.cs_add_button.clicked.connect(self.define_cs_cb)
        self.cs_scan_button.clicked.connect(self.scan_cs_cb)
        self.cs_set_button.clicked.connect(self.set_cs_cb)
        self.cs_show_button.clicked.connect(self.show_cs_cb)
        self.cs_remove_button.clicked.connect(self.remove_cs_cb)
        self.cs_list_button.clicked.connect(self.get_list_cs_cb)
        self.pe_add_button.clicked.connect(self.add_pe_cb)
        self.pe_hide_button.clicked.connect(self.hide_pe_cb)
        self.pe_unhide_button.clicked.connect(self.unhide_pe_cb)
        self.pe_remove_button.clicked.connect(self.remove_pe_cb)
        self.pe_monitor_button.clicked.connect(self.monitor_pe_cb)

    def open_services(self):
        """Open ROS services."""
        # ROS services and topics
        self.connect       = rospy.ServiceProxy('connect', Trigger)
        self.disconnect    = rospy.ServiceProxy('disconnect', Trigger)
        self.start_proj    = rospy.ServiceProxy('projection_start', Trigger)
        self.stop_proj     = rospy.ServiceProxy('projection_stop', Trigger)
        self.manual_cs     = rospy.ServiceProxy('define_coordinate_system', CoordinateSystem)
        self.auto_cs       = rospy.ServiceProxy('search_targets', CoordinateSystem)
        self.get_cs_list   = rospy.ServiceProxy('coordinate_system_list', CoordinateSystemList)
        self.set_cs        = rospy.ServiceProxy('set_coordinate_system', CoordinateSystemName)
        self.show_cs       = rospy.ServiceProxy('show_active_coordinate_system', CoordinateSystemShow)
        self.rem_cs        = rospy.ServiceProxy('remove_coordinate_system', CoordinateSystemName)
        self.hide_figure   = rospy.ServiceProxy('hide_projection_element', ProjectionElement)
        self.unhide_figure = rospy.ServiceProxy('unhide_projection_element', ProjectionElement)
        self.remove_figure = rospy.ServiceProxy('remove_projection_element', ProjectionElement)
        self.monit_figure  = rospy.ServiceProxy('monitor_projection_element', ProjectionElement)

        self.add_proj_elem = rospy.Publisher('add_projection_element', Figure, queue_size=10)
    
    def connect_cb(self):
        """Button callback to call: connect service."""
        resp = self.connect()
        rospy.loginfo("Service response: \n{}".format(resp))
        
        if resp.success:
            self.status.setPixmap(QtGui.QPixmap(self.ON_LED))
        else:
            self.error_msg("Connection error",resp.message)

    def disconnect_cb(self):
        """Button callback to call: disconnect service."""
        resp = self.disconnect()
        rospy.loginfo("Service response: \n{}".format(resp))

        if resp.success:
            self.status.setPixmap(QtGui.QPixmap(self.OFF_LED))
        else:
            self.error_msg("Disconnection error",resp.message)

    def start_proj_cb(self): 
        """Button callback to call: start projection service."""
        resp = self.start_proj()
        rospy.loginfo("Service response: \n{}".format(resp))

        if resp.success:
            self.status_2.setPixmap(QtGui.QPixmap(self.ON_LED))
        else:
            self.error_msg("Projection error",resp.message)

    def stop_proj_cb(self):
        """Button callback to call: stop projection service."""
        resp = self.stop_proj()
        rospy.loginfo("Service response: \n{}".format(resp))

        if resp.success:
            self.status_2.setPixmap(QtGui.QPixmap(self.OFF_LED))
        else:
            self.error_msg("Projection error",resp.message)
    
    def get_list_cs_cb(self):
        """Button callback to call: get coordinate system list service."""
        resp = self.get_cs_list()
        rospy.loginfo("Service response: \n{}".format(resp))
        if not resp.success:
            self.error_msg("Coordinate System Manager error",resp.message)

    def create_req(self):
        """Read parameters from text boxes to define a new coordinate system.
        
        Returns:
            object: object with the parameters read from the GUI
        """
        coord_sys = CoordinateSystemRequest()
        coord_sys.name = str(self.cs_name.text())
        coord_sys.distance = float(self.cs_dist.text())
        coord_sys.resolution = float(self.cs_res.text())
        coord_sys.T0 = Point(float(self.t1_x.text()),float(self.t1_y.text()),float(self.t1_z.text()))
        coord_sys.P.append(Point(float(self.p1_x.text()),float(self.p1_y.text()),float(self.p1_z.text())))
        coord_sys.P.append(Point(float(self.p2_x.text()),float(self.p2_y.text()),float(self.p2_z.text())))
        coord_sys.P.append(Point(float(self.p3_x.text()),float(self.p3_y.text()),float(self.p3_z.text())))
        coord_sys.P.append(Point(float(self.p4_x.text()),float(self.p4_y.text()),float(self.p4_z.text())))
        return coord_sys

    def define_cs_cb(self):
        """Button callback to call: define coordinate system service."""
        try:
            resp = self.manual_cs(self.create_req())
            rospy.loginfo("Service response: \n{}".format(resp))
            if not resp.success:
                self.error_msg("Coordinate System Menu error",resp.message)

        except Exception as e:
            self.error_msg("Coordinate System Menu error",e)

    def scan_cs_cb(self):
        """Button callback to call: scan targets service."""
        try:
            resp = self.auto_cs(self.create_req())
            rospy.loginfo("Service response: \n{}".format(resp))
            if not resp.success:
                self.error_msg("Coordinate System Menu error",resp.message)

        except Exception as e:
            self.error_msg("Coordinate System Menu error",e)

    def set_cs_cb(self):
        """Button callback to call: set coordinate system service."""
        try:
            coord_sys      = CoordinateSystemNameRequest()
            coord_sys.name = str(self.cs_name_handler.text())

            resp = self.set_cs(coord_sys)
            rospy.loginfo("Service response: \n{}".format(resp))
            if not resp.success:
                self.error_msg("Coordinate System Manager error",resp.message)

        except Exception as e:
            self.error_msg("Coordinate System Menu error",e)

    def show_cs_cb(self):
        """Button callback to call: show coordinate system service."""
        try:
            coord_sys      = CoordinateSystemShowRequest()
            coord_sys.secs = int(self.cs_show_secs.text())

            resp = self.show_cs(coord_sys)
            rospy.loginfo("Service response: \n{}".format(resp))
            if not resp.success:
                self.error_msg("Coordinate System Manager error",resp.message)

        except Exception as e:
            self.error_msg("Coordinate System Menu error",e)

    def remove_cs_cb(self):
        """Button callback to call: remove coordinate system service."""
        try:
            coord_sys      = CoordinateSystemNameRequest()
            coord_sys.name = str(self.cs_name_handler.text())

            resp = self.rem_cs(coord_sys)
            rospy.loginfo("Service response: \n{}".format(resp))
            if not resp.success:
                self.error_msg("Coordinate System Manager error",resp.message)

        except Exception as e:
            self.error_msg("Coordinate System Menu error",e)

    def update_entries(self):
        """Callback of drop down list object: update input text boxes from projection element GUI submenu."""
        self.init_pe_menu()
        figure = self.pe_menu.currentText()

        if figure == "polyline":
            self.length.setEnabled(True)
            self.length.setVisible(True)
            self.pe_length.setEnabled(True)
            self.pe_length.setVisible(True)
            self.mm_len.setEnabled(True)
            self.mm_len.setVisible(True)
            self.start_ang.setEnabled(True)
            self.start_ang.setVisible(True)
            self.pe_angle.setEnabled(True)
            self.pe_angle.setVisible(True)
            self.deg_start.setEnabled(True)
            self.deg_start.setVisible(True)
        elif figure == "circle":
            self.length.setEnabled(True)
            self.length.setVisible(True)
            self.pe_length.setEnabled(True)
            self.pe_length.setVisible(True)
            self.mm_len.setEnabled(True)
            self.mm_len.setVisible(True)
        elif figure == "oval":
            self.length.setEnabled(True)
            self.length.setVisible(True)
            self.pe_length.setEnabled(True)
            self.pe_length.setVisible(True)
            self.mm_len.setEnabled(True)
            self.mm_len.setVisible(True)
            self.start_ang.setEnabled(True)
            self.start_ang.setVisible(True)
            self.pe_angle.setEnabled(True)
            self.pe_angle.setVisible(True)
            self.deg_start.setEnabled(True)
            self.deg_start.setVisible(True)
            self.height.setEnabled(True)
            self.height.setVisible(True)
            self.pe_height.setEnabled(True)
            self.pe_height.setVisible(True)
            self.mm_height.setEnabled(True)
            self.mm_height.setVisible(True)
        elif figure == "arc":
            self.length.setEnabled(True)
            self.length.setVisible(True)
            self.pe_length.setEnabled(True)
            self.pe_length.setVisible(True)
            self.mm_len.setEnabled(True)
            self.mm_len.setVisible(True)
            self.start_ang.setEnabled(True)
            self.start_ang.setVisible(True)
            self.pe_angle.setEnabled(True)
            self.pe_angle.setVisible(True)
            self.deg_start.setEnabled(True)
            self.deg_start.setVisible(True)
            self.end_ang.setEnabled(True)
            self.end_ang.setVisible(True)
            self.pe_end_angle.setEnabled(True)
            self.pe_end_angle.setVisible(True)
            self.deg_end.setEnabled(True)
            self.deg_end.setVisible(True)
        elif figure == "text":
            self.start_ang.setEnabled(True)
            self.start_ang.setVisible(True)
            self.pe_angle.setEnabled(True)
            self.pe_angle.setVisible(True)
            self.deg_start.setEnabled(True)
            self.deg_start.setVisible(True)
            self.text.setEnabled(True)
            self.text.setVisible(True)
            self.pe_text.setEnabled(True)
            self.pe_text.setVisible(True)
            self.char_spac.setEnabled(True)
            self.char_spac.setVisible(True)
            self.pe_char_space.setEnabled(True)
            self.pe_char_space.setVisible(True)
            self.height.setEnabled(True)
            self.height.setVisible(True)
            self.pe_height.setEnabled(True)
            self.pe_height.setVisible(True)
            self.mm_height.setEnabled(True)
            self.mm_height.setVisible(True)

    def add_pe_cb(self):
        """Button callback to pub: add projection element topic."""
        figure = self.pe_menu.currentText()

        if figure == "polyline":
            self.line_def()
        elif figure == "circle":
            self.circle_def()
        elif figure == "oval":
            self.oval_def()
        elif figure == "arc":
            self.arc_def()
        elif figure == "text":
            self.text_def()
        else:
            self.error_msg("Projection element definition error","No projection element type selected.")

    def line_def(self):
        """Publish new line projection element on topic."""
        try:
            line                  = Figure()
            line.figure_type      = int(0)
            line.projection_group = str(self.pe_group.text())
            line.figure_name      = str(self.pe_id.text())
            line.position.x       = float(self.pe_x.text())
            line.position.y       = float(self.pe_y.text())
            line.size.append(float(self.pe_length.text()))
            line.angle.append(float(self.pe_angle.text()))
            self.add_proj_elem.publish(line)

        except Exception as e:
            self.error_msg("Projection element Menu error",e)

    def circle_def(self):
        """Publish new circle projection element on topic."""
        try:
            circle                  = Figure()
            circle.figure_type      = int(1)
            circle.projection_group = str(self.pe_group.text())
            circle.figure_name      = str(self.pe_id.text())
            circle.position.x       = float(self.pe_x.text())
            circle.position.y       = float(self.pe_y.text())
            circle.size.append(float(self.pe_length.text()))

            self.add_proj_elem.publish(circle)

        except Exception as e:
            self.error_msg("Projection element Menu error",e)

    def arc_def(self):
        """Publish new arc projection element on topic."""
        try:
            arc                  = Figure()
            arc.figure_type      = int(2)
            arc.projection_group = str(self.pe_group.text())
            arc.figure_name      = str(self.pe_id.text())
            arc.position.x       = float(self.pe_x.text())
            arc.position.y       = float(self.pe_y.text())
            arc.size.append(float(self.pe_length.text()))
            arc.angle.append(float(self.pe_angle.text()))
            arc.angle.append(float(self.pe_end_angle.text()))

            self.add_proj_elem.publish(arc)

        except Exception as e:
            self.error_msg("Projection element Menu error",e)

    def oval_def(self):
        """Publish new ellipse projection element on topic."""
        try:
            oval                  = Figure()
            oval.figure_type      = int(3)
            oval.projection_group = str(self.pe_group.text())
            oval.figure_name      = str(self.pe_id.text())
            oval.position.x       = float(self.pe_x.text())
            oval.position.y       = float(self.pe_y.text())
            oval.size.append(float(self.pe_length.text()))
            oval.size.append(float(self.pe_height.text()))
            oval.angle.append(float(self.pe_angle.text()))

            self.add_proj_elem.publish(oval)

        except Exception as e:
            self.error_msg("Projection element Menu error",e)

    def text_def(self):
        """Publish new text projection element on topic."""
        try:
            text                  = Figure()
            text.figure_type      = int(4)
            text.projection_group = str(self.pe_group.text())
            text.figure_name      = str(self.pe_id.text())
            text.position.x       = float(self.pe_x.text())
            text.position.y       = float(self.pe_y.text())
            text.size.append(float(self.pe_height.text()))
            text.size.append(float(self.pe_char_space.text()))
            text.angle.append(float(self.pe_angle.text()))
            text.text             = str(self.pe_text.text())

            self.add_proj_elem.publish(text)

        except Exception as e:
            self.error_msg("Projection element Menu error",e)

    def hide_pe_cb(self):
        """Button callback to call: hide projection element service."""
        try:
            proj_elem                  = ProjectionElementRequest()
            proj_elem.figure_type      = int(self.pe_menu_2.currentIndex())
            proj_elem.projection_group = str(self.pe_group_2.text())
            proj_elem.figure_name      = str(self.pe_id_2.text())

            if 0 <= proj_elem.figure_type <= 4:
                resp = self.hide_figure(proj_elem)
                rospy.loginfo("Service response: \n{}".format(resp))
                if not resp.success:
                    self.error_msg("Projection element Manager error",resp.message)
            else:
                self.error_msg("Projection element Manager error","Select a projection element type")

        except Exception as e:
            self.error_msg("Projection element Menu error",e)

    def unhide_pe_cb(self):
        """Button callback to call: unhide projection element service."""
        try:
            proj_elem                  = ProjectionElementRequest()
            proj_elem.figure_type      = int(self.pe_menu_2.currentIndex())
            proj_elem.projection_group = str(self.pe_group_2.text())
            proj_elem.figure_name      = str(self.pe_id_2.text())
            
            if 0 <= proj_elem.figure_type <= 4:
                resp = self.unhide_figure(proj_elem)
                rospy.loginfo("Service response: \n{}".format(resp))
                if not resp.success:
                    self.error_msg("Projection element Manager error",resp.message)
            else:
                self.error_msg("Projection element Manager error","Select a projection element type")
        
        except Exception as e:
            self.error_msg("Projection element Menu error",e)

    def remove_pe_cb(self):
        """Button callback to call: remove projection element service."""
        try:
            proj_elem                  = ProjectionElementRequest()
            proj_elem.figure_type      = int(self.pe_menu_2.currentIndex())
            proj_elem.projection_group = str(self.pe_group_2.text())
            proj_elem.figure_name      = str(self.pe_id_2.text())
            
            if 0 <= proj_elem.figure_type <= 4:
                resp = self.remove_figure(proj_elem)
                rospy.loginfo("Service response: \n{}".format(resp))
                if not resp.success:
                    self.error_msg("Projection element Manager error",resp.message)
            else:
                self.error_msg("Projection element Manager error","Select a projection element type")

        except Exception as e:
            self.error_msg("Projection element Menu error",e)

    def monitor_pe_cb(self):
        """Button callback to call: monitor projection element service."""
        try:
            proj_elem                  = ProjectionElementRequest()
            proj_elem.figure_type      = int(self.pe_menu_2.currentIndex())
            proj_elem.projection_group = str(self.pe_group_2.text())
            proj_elem.figure_name      = str(self.pe_id_2.text())
            
            if 0 <= proj_elem.figure_type <= 4:
                resp = self.monit_figure(proj_elem)
                rospy.loginfo("Service response: \n{}".format(resp))
                if not resp.success:
                    self.error_msg("Projection element Manager error",resp.message)
            else:
                self.error_msg("Projection element Manager error","Select a projection element type")
        
        except Exception as e:
            self.error_msg("Projection element Menu error",e)

    def error_msg(self,title,e):
        """Run and set message of pop-up error window."""
        self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
        self.msg_box.setWindowTitle(title)
        self.msg_box.setText("Error: {}".format(e))
        self.msg_box.exec_()

    def closeEvent(self, event):
        """Stop GUI on press close key button."""
        close = QtWidgets.QMessageBox()
        close.setText("Are you sure you want to quit?")
        close.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel)
        closed = close.exec_()

        if closed == QtWidgets.QMessageBox.Yes:
            event.accept()
        else:
            event.ignore()
