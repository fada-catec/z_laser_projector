#!/usr/bin/env python3

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


import rospy
import sys
from PyQt5 import QtWidgets

from z_laser_gui.zlp_gui import MyWindow


if __name__ == "__main__":

    rospy.init_node('zlp_gui_node')

    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    window.show()

    result = app.exec_()

    sys.exit(result)

    rospy.spin()