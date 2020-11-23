#!/usr/bin/env python
# Copyright (c) 2020 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import os

import rospkg
import rospy
from pilz_msgs.msg import OperationModes
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtWidgets import QWidget

GREEN = "green"
RED = "red"


class PilzStatusIndicatorWidget(QWidget):
    def __init__(self, serial_number):  # pragma no cover
        """Initializes the widget. Widget content will be loaded from 
        `PilzStatusIndicatorRqt.ui`.

        :param serial_number: A serial number to differentiate multiple
                              instances of the same widget."""
        super(PilzStatusIndicatorWidget, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', 'PilzStatusIndicatorRqt.ui')
        loadUi(ui_file, self)
        self.setObjectName('PilzStatusIndicatorRqtUi')

        # Checking if widget is loaded correctly from ui file
        assert self.labelROS, "ROS label must be loaded from ui file"
        assert self.labelHW, "HW label must be loaded from ui file"
        assert self.labelOM, "OM label must be loaded from ui file"
        assert self.labelOM_text, "OM text label must be loaded from ui file"
        assert self.barSpeed, "barSpeed must be loaded from ui file"

        # Prepare ui elements
        self.labelOM.setScaledContents(True)

        # Show windowTitle on left-top of each plugin.
        if serial_number > 1:
            self.setWindowTitle(
                self.windowTitle() + (' (%d)' % serial_number))

    def _set_label_status_view(self, label, status):
        if status:
            label.setStyleSheet("QLabel { background-color: %s }" % GREEN)
        else:
            label.setStyleSheet("QLabel { background-color: %s }" % RED)

    def set_ROS_status(self, status):
        """Sets the HW status to be displayed in the widget, using
        a red or green LED.

        :param status: The status to set:
                       False will be red, True will be green."""
        self._set_label_status_view(self.labelROS, status)

    def set_HW_status(self, status):
        """Sets the HW status to be displayed in the widget, using
        a red or green LED.

        :param status: The status to set:
                       False will be red, True will be green."""
        self._set_label_status_view(self.labelHW, status)

    def set_operation_mode(self, mode, _qpixmap_class=QPixmap):
        """Sets the operation mode to be displayed in the widget, influencing
        both the shown image and the text beneath it.

        :param mode: The mode to be set of type
                     `pilz_msgs.msg.OperationModes`.
        :param _qpixmap_class: (Internal use only)"""
        if mode == OperationModes.AUTO:
            icon_name = 'auto'
        elif mode == OperationModes.T1:
            icon_name = 't1'
        elif mode == OperationModes.T2:
            icon_name = 't2'
        else:  # mode == OperationModes.UNKNOWN
            icon_name = 'unknown'        
        icon_path = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', icon_name + '.png')
        pixmap = _qpixmap_class(icon_path)
        self.labelOM.setPixmap(pixmap)
        self.labelOM_text.setText(icon_name.capitalize())

    def set_speed(self, val):
        """Sets the speed override to be displayed in the widget, influencing
        the progress bar and the textual percentage within it.

        :param val: The speed override as a value between 0 and 1."""
        if val > 1 or val < 0:  # expecting val = 0...1
            rospy.logwarn(
                "expecting speed value between 0 and 1, got {} !".format(val))
            if val > 1:
                val = 1
            else:
                val = 0
        self.barSpeed.setValue(100. * val)
