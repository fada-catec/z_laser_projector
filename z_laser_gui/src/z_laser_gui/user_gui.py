#!/usr/bin/env python3

import sys
import os
import rospy


from PyQt5 import QtCore, QtGui, QtWidgets, uic
# from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QMessageBox

# qtCreatorFile = "test3.ui" # Enter file here.
# Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)

from main_window import Ui_MainWindow


from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Point
from z_laser_msgs.msg import Line, Curve, Text
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

        self.msg_box = QtWidgets.QMessageBox()
        # self.msg_box.setStandardButtons(QtWidgets.QMessageBox.Ok)
        # self.msg_box.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)

        # LEDS
        self.status.setPixmap(QtGui.QPixmap(CONNECT_OFF_LED))
        self.status_2.setPixmap(QtGui.QPixmap(PROJECT_OFF_LED))

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
        self.add_line      = rospy.Publisher('add_line', Line, queue_size=10)
        self.add_curve     = rospy.Publisher('add_curve', Curve, queue_size=10)
        self.add_text      = rospy.Publisher('add_text', Text, queue_size=10)


        self.projector_connected = False
        self.set_up_gui()

    def set_up_gui(self):

        rospy.wait_for_service("connect")
        self.projector_connected = rospy.get_param('projector_connected')

        if self.projector_connected:
            self.status.setPixmap(QtGui.QPixmap(CONNECT_ON_LED))
        else:
            self.status.setPixmap(QtGui.QPixmap(CONNECT_OFF_LED))
    
    def connect_cb(self):
        
        resp = self.connect()
        rospy.loginfo("Service response: \n{}".format(resp))
        
        if resp.success:
            self.status.setPixmap(QtGui.QPixmap(CONNECT_ON_LED))
        else:
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("Connection error")
            self.msg_box.setText(resp.message)
            # self.msg_box.setInformativeText("Please disconnect projector first.")
            # self.msg_box.setDetailedText("The details are as follows:")
            self.msg_box.exec_()

    def disconnect_cb(self):

        resp = self.disconnect()
        rospy.loginfo("Service response: \n{}".format(resp))

        if resp.success:
            self.status.setPixmap(QtGui.QPixmap(CONNECT_OFF_LED))
        else:
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("Disconnection error")
            self.msg_box.setText(resp.message)
            self.msg_box.exec_()

    def start_proj_cb(self): 
        
        resp = self.start_proj()
        rospy.loginfo("Service response: \n{}".format(resp))

        if resp.success:
            self.status_2.setPixmap(QtGui.QPixmap(PROJECT_ON_LED))
        else:
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("Projection error")
            self.msg_box.setText(resp.message)
            self.msg_box.exec_()

    def stop_proj_cb(self):

        resp = self.stop_proj()
        rospy.loginfo("Service response: \n{}".format(resp))

        if resp.success:
            self.status_2.setPixmap(QtGui.QPixmap(PROJECT_OFF_LED))
        else:
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("Projection error")
            self.msg_box.setText(resp.message)
            self.msg_box.exec_()
    
    def get_list_cs_cb(self):

        resp = self.get_cs_list()
        rospy.loginfo("Service response: \n{}".format(resp))
        if not resp.success:
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("An error ocurred")
            self.msg_box.setText(resp.message)
            self.msg_box.exec_()

    def define_cs_cb(self):

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
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("An error ocurred")
            self.msg_box.setText(resp.message)
            self.msg_box.exec_()

    def scan_cs_cb(self):
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
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("An error ocurred")
            self.msg_box.setText(resp.message)
            self.msg_box.exec_()

    def set_cs_cb(self):

        coord_sys      = CoordinateSystemNameRequest()
        coord_sys.name = str(self.cs_name_handler.text())

        resp = self.set_cs(coord_sys)
        rospy.loginfo("Service response: \n{}".format(resp))
        if not resp.success:
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("An error ocurred")
            self.msg_box.setText(resp.message)
            self.msg_box.exec_()

    def show_cs_cb(self):
        
        # self.msg_box.setIcon(QtWidgets.QMessageBox.Information)
        # self.msg_box.setWindowTitle("Projecting")
        # self.msg_box.setText("Coordinate System is going to be projected.")
        # self.msg_box.setInformativeText("Please wait until it finishes.")
        # self.msg_box.exec_()

        coord_sys      = CoordinateSystemShowRequest()
        coord_sys.secs = int(self.cs_show_secs.text())

        resp = self.show_cs(coord_sys)
        rospy.loginfo("Service response: \n{}".format(resp))
        if not resp.success:
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("An error ocurred")
            self.msg_box.setText(resp.message)
            self.msg_box.exec_()

    def remove_cs_cb(self):

        coord_sys      = CoordinateSystemNameRequest()
        coord_sys.name = str(self.cs_name_handler.text())

        resp = self.rem_cs(coord_sys)
        rospy.loginfo("Service response: \n{}".format(resp))
        if not resp.success:
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("An error ocurred")
            self.msg_box.setText(resp.message)
            self.msg_box.exec_()

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
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("Projection element definition error")
            self.msg_box.setText("No projection element type selected.")
            self.msg_box.setInformativeText("Please choose a projection element type.")
            # self.msg_box.setDetailedText("The details are as follows:")
            self.msg_box.exec_()

    def line_def(self):

        line                  = Line()
        line.projection_group = str(self.pe_group.text())
        line.figure_name      = str(self.pe_id.text())
        line.x                = float(self.pe_x.text())
        line.y                = float(self.pe_y.text())
        line.angle            = float(self.pe_angle.text())
        line.length           = float(self.pe_length.text())

        self.add_line.publish(line)

    def circle_def(self):

        circle                  = Curve()
        circle.projection_group = str(self.pe_group.text())
        circle.figure_name      = str(self.pe_id.text())
        circle.curve_type       = "circle"
        circle.x                = float(self.pe_x.text())
        circle.y                = float(self.pe_y.text())
        circle.length           = float(self.pe_length.text())

        self.add_curve.publish(circle)

    def oval_def(self):

        oval                  = Curve()
        oval.projection_group = str(self.pe_group.text())
        oval.figure_name      = str(self.pe_id.text())
        oval.curve_type       = "oval"
        oval.x                = float(self.pe_x.text())
        oval.y                = float(self.pe_y.text())
        oval.length           = float(self.pe_length.text())
        oval.height           = float(self.pe_height.text())

        self.add_curve.publish(oval)

    def arc_def(self):

        arc                  = Curve()
        arc.projection_group = str(self.pe_group.text())
        arc.figure_name      = str(self.pe_id.text())
        arc.curve_type       = "arc"
        arc.x                = float(self.pe_x.text())
        arc.y                = float(self.pe_y.text())
        arc.angle            = float(self.pe_angle.text())
        arc.end_angle        = float(self.pe_end_angle.text())
        arc.length           = float(self.pe_length.text())

        self.add_curve.publish(arc)

    def text_def(self):

        text                  = Text()
        text.projection_group = str(self.pe_group.text())
        text.figure_name      = str(self.pe_id.text())
        text.text             = str(self.pe_text.text())
        text.x                = float(self.pe_x.text())
        text.y                = float(self.pe_y.text())
        text.angle            = float(self.pe_angle.text())
        text.height           = float(self.pe_height.text())
        text.char_spacing     = float(self.pe_char_space.text())

        self.add_text.publish(text)

    def hide_pe_cb(self):

        proj_elem                  = ProjectionElementRequest()
        proj_elem.figure_type      = self.pe_menu_2.currentText()
        proj_elem.projection_group = str(self.pe_group_2.text())
        proj_elem.figure_name      = str(self.pe_id_2.text())

        resp = self.hide_figure(proj_elem)
        rospy.loginfo("Service response: \n{}".format(resp))
        if not resp.success:
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("An error ocurred")
            self.msg_box.setText(resp.message)
            self.msg_box.exec_()

    def unhide_pe_cb(self):

        proj_elem                  = ProjectionElementRequest()
        proj_elem.figure_type      = self.pe_menu_2.currentText()
        proj_elem.projection_group = str(self.pe_group_2.text())
        proj_elem.figure_name      = str(self.pe_id_2.text())
        
        resp = self.unhide_figure(proj_elem)
        rospy.loginfo("Service response: \n{}".format(resp))
        if not resp.success:
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("An error ocurred")
            self.msg_box.setText(resp.message)
            self.msg_box.exec_()

    def remove_pe_cb(self):

        proj_elem                  = ProjectionElementRequest()
        proj_elem.figure_type      = self.pe_menu_2.currentText()
        proj_elem.projection_group = str(self.pe_group_2.text())
        proj_elem.figure_name      = str(self.pe_id_2.text())
        
        resp = self.remove_figure(proj_elem)
        rospy.loginfo("Service response: \n{}".format(resp))
        if not resp.success:
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("An error ocurred")
            self.msg_box.setText(resp.message)
            self.msg_box.exec_()

    def monitor_pe_cb(self):
        
        proj_elem                  = ProjectionElementRequest()
        proj_elem.figure_type      = self.pe_menu_2.currentText()
        proj_elem.projection_group = str(self.pe_group_2.text())
        proj_elem.figure_name      = str(self.pe_id_2.text())
        
        resp = self.monit_figure(proj_elem)
        rospy.loginfo("Service response: \n{}".format(resp))
        if not resp.success:
            self.msg_box.setIcon(QtWidgets.QMessageBox.Critical)
            self.msg_box.setWindowTitle("An error ocurred")
            self.msg_box.setText(resp.message)
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