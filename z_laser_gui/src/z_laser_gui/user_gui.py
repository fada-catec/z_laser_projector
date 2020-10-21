#!/usr/bin/env python3

import sys
import os
import rospy


from PyQt5 import QtCore, QtGui, QtWidgets, uic
# from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QMessageBox

from main_window import Ui_MainWindow

from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Point
from z_laser_msgs.msg import Figure
from z_laser_msgs.srv import CoordinateSystem, CoordinateSystemRequest, CoordinateSystemResponse
from z_laser_msgs.srv import CoordinateSystemName, CoordinateSystemNameRequest, CoordinateSystemNameResponse
from z_laser_msgs.srv import CoordinateSystemShow, CoordinateSystemShowRequest, CoordinateSystemShowResponse
from z_laser_msgs.srv import CoordinateSystemList, CoordinateSystemListResponse
from z_laser_msgs.srv import ProjectionElement, ProjectionElementRequest, ProjectionElementResponse


CONNECT_ON_LED  = os.path.dirname(os.path.abspath(__file__)) + "/icons/ON.png"
CONNECT_OFF_LED = os.path.dirname(os.path.abspath(__file__)) + "/icons/OFF.png"
PROJECT_ON_LED  = os.path.dirname(os.path.abspath(__file__)) + "/icons/ON.png"
PROJECT_OFF_LED = os.path.dirname(os.path.abspath(__file__)) + "/icons/OFF.png"

class MyWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        # USER GUI setup
        QtWidgets.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)

        self.projector_connected = False
        self.set_up_gui()

    def set_up_gui(self):

        # LEDS
        self.status.setPixmap(QtGui.QPixmap(CONNECT_OFF_LED))
        self.status_2.setPixmap(QtGui.QPixmap(PROJECT_OFF_LED))

        # Msgs box
        self.msg_box = QtWidgets.QMessageBox()
        # self.msg_box.setStandardButtons(QtWidgets.QMessageBox.Ok)
        # self.msg_box.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)

        self.init_pe_menu()

        self.set_buttons()

        self.open_services()

        rospy.wait_for_service("connect")
        self.projector_connected = rospy.get_param('projector_connected')

        if self.projector_connected:
            self.status.setPixmap(QtGui.QPixmap(CONNECT_ON_LED))
        else:
            self.status.setPixmap(QtGui.QPixmap(CONNECT_OFF_LED))

    def init_pe_menu(self):

        # Set ComboBox callback
        self.pe_menu.currentIndexChanged.connect(self.update_entries)

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
        # GUI Buttons
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
        self.hide_figure   = rospy.ServiceProxy('hide_figure', ProjectionElement)
        self.unhide_figure = rospy.ServiceProxy('unhide_figure', ProjectionElement)
        self.remove_figure = rospy.ServiceProxy('remove_figure', ProjectionElement)
        self.monit_figure  = rospy.ServiceProxy('monitor_figure', ProjectionElement)

        self.add_proj_elem = rospy.Publisher('add_projection_element', Figure, queue_size=10)
    
    def connect_cb(self):
        
        resp = self.connect()
        rospy.loginfo("Service response: \n{}".format(resp))
        
        if resp.success:
            self.status.setPixmap(QtGui.QPixmap(CONNECT_ON_LED))
        else:
            self.error_msg("Connection error",resp.message)

    def disconnect_cb(self):

        resp = self.disconnect()
        rospy.loginfo("Service response: \n{}".format(resp))

        if resp.success:
            self.status.setPixmap(QtGui.QPixmap(CONNECT_OFF_LED))
        else:
            self.error_msg("Disconnection error",resp.message)

    def start_proj_cb(self): 
        
        resp = self.start_proj()
        rospy.loginfo("Service response: \n{}".format(resp))

        if resp.success:
            self.status_2.setPixmap(QtGui.QPixmap(PROJECT_ON_LED))
        else:
            self.error_msg("Projection error",resp.message)

    def stop_proj_cb(self):

        resp = self.stop_proj()
        rospy.loginfo("Service response: \n{}".format(resp))

        if resp.success:
            self.status_2.setPixmap(QtGui.QPixmap(PROJECT_OFF_LED))
        else:
            self.error_msg("Projection error",resp.message)
    
    def get_list_cs_cb(self):

        resp = self.get_cs_list()
        rospy.loginfo("Service response: \n{}".format(resp))
        if not resp.success:
            self.error_msg("Coordinate System Manager error",resp.message)

    def define_cs_cb(self):

        try:
            coord_sys            = CoordinateSystemRequest()
            coord_sys.name       = str(self.cs_name.text())
            coord_sys.distance   = float(self.cs_dist.text())
            coord_sys.P1         = Point(float(self.p1_x.text()),float(self.p1_y.text()),float(self.p1_z.text()))
            coord_sys.P2         = Point(float(self.p2_x.text()),float(self.p2_y.text()),float(self.p2_z.text()))
            coord_sys.P3         = Point(float(self.p3_x.text()),float(self.p3_y.text()),float(self.p3_z.text()))
            coord_sys.P4         = Point(float(self.p4_x.text()),float(self.p4_y.text()),float(self.p4_z.text()))
            coord_sys.T1         = Point(float(self.t1_x.text()),float(self.t1_y.text()),float(self.t1_z.text()))
            coord_sys.resolution = float(self.cs_res.text())

            resp = self.manual_cs(coord_sys)
            rospy.loginfo("Service response: \n{}".format(resp))
            if not resp.success:
                self.error_msg("Coordinate System Menu error",resp.message)

        except Exception as e:
            self.error_msg("Coordinate System Menu error",e)

    def scan_cs_cb(self):
        
        try:
            coord_sys            = CoordinateSystemRequest()
            coord_sys.name       = str(self.cs_name.text())
            coord_sys.distance   = float(self.cs_dist.text())
            coord_sys.P1         = Point(float(self.p1_x.text()),float(self.p1_y.text()),float(self.p1_z.text()))
            coord_sys.P2         = Point(float(self.p2_x.text()),float(self.p2_y.text()),float(self.p2_z.text()))
            coord_sys.P3         = Point(float(self.p3_x.text()),float(self.p3_y.text()),float(self.p3_z.text()))
            coord_sys.P4         = Point(float(self.p4_x.text()),float(self.p4_y.text()),float(self.p4_z.text()))
            coord_sys.T1         = Point(float(self.t1_x.text()),float(self.t1_y.text()),float(self.t1_z.text()))
            coord_sys.resolution = float(self.cs_res.text())

            resp = self.auto_cs(coord_sys)
            rospy.loginfo("Service response: \n{}".format(resp))
            if not resp.success:
                self.error_msg("Coordinate System Menu error",resp.message)

        except Exception as e:
            self.error_msg("Coordinate System Menu error",e)

    def set_cs_cb(self):

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
        
        self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
        self.msg_box.setWindowTitle(title)
        self.msg_box.setText("Error: {}".format(e))
        # self.msg_box.setInformativeText("error: {}".format(e))
        self.msg_box.exec_()

    def closeEvent(self, event):
        close = QtWidgets.QMessageBox()
        close.setText("Are you sure you want to quit?")
        close.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel)
        close = close.exec()

        # quit = QtWidgets.QAction("Quit", self)
        # quit.triggered.connect(self.close)

        if close == QtWidgets.QMessageBox.Yes:
            # QtWidgets.QApplication.quit()
            event.accept()
        else:
            event.ignore()

if __name__ == "__main__":

    rospy.init_node('zlp_gui_node')

    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    window.show()

    result = app.exec_()

    sys.exit(result)

    rospy.spin()