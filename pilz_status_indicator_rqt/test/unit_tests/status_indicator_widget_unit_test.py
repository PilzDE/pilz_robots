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
import sys
import unittest

import rospkg
import rospy
from mock import ANY, MagicMock, create_autospec
from pilz_msgs.msg import OperationModes
from PyQt5.QtGui import QPixmap
from python_qt_binding.QtWidgets import QApplication, QMainWindow

from pilz_status_indicator_rqt.status_indicator_widget import (
    GREEN, RED, PilzStatusIndicatorWidget)


class TestablePilzStatusIndicatorWidget(PilzStatusIndicatorWidget):
    """
    Needed to skip TestStatusIndicatorWidget.__init__, especially the super
    call within. Was not able to mock. Better solution welcome.
    """

    def __init__(self, serial_number):
        pass


class TestStatusIndicatorWidget(unittest.TestCase):
    """
    These tests ensure that status changes for the widget make the correct
    calls internally to display these information correctly.
    """

    def setUp(self):
        """initializing a widget to be tested."""
        self.psi = TestablePilzStatusIndicatorWidget(0)

    def test_set_ROS_status(self):
        """Testing whether the LED for the ROS status is turning correctly
        red and green."""
        # Preparing mocks within the widget
        self.psi.labelROS = MagicMock()
        self.psi.labelROS.setStyleSheet = MagicMock()

        # Testing if light turns green for status True
        self.psi.set_ROS_status(True)
        self.psi.labelROS.setStyleSheet.assert_called_with(
            "QLabel { background-color: %s }" % GREEN)

        # Testing if light turns red for status False
        self.psi.set_ROS_status(False)
        self.psi.labelROS.setStyleSheet.assert_called_with(
            "QLabel { background-color: %s }" % RED)

    def test_set_HW_status(self):
        """Testing whether the LED for the HW status is turning correctly
        red and green."""
        # Preparing mocks within the widget
        self.psi.labelHW = MagicMock()
        self.psi.labelHW.setStyleSheet = MagicMock()

        # Testing if light turns green for status True
        self.psi.set_HW_status(True)
        self.psi.labelHW.setStyleSheet.assert_called_with(
            "QLabel { background-color: %s }" % GREEN)

        # Testing if light turns red for status False
        self.psi.set_HW_status(False)
        self.psi.labelHW.setStyleSheet.assert_called_with(
            "QLabel { background-color: %s }" % RED)

    def test_set_operation_modes(self):
        """Testing whether setting the operation mode loads the correct icon
        and displays the right text."""
        # Preparing mocks within the widget
        Mock_qpixmap = create_autospec(QPixmap)
        self.psi.labelOM = MagicMock()
        self.psi.labelOM_text = MagicMock()
        self.psi.labelOM.setPixmap = MagicMock()
        self.psi.labelOM_text.setText = MagicMock()

        # Operation mode Auto should load the correct icon and display the
        # correct text.
        icon_path_auto = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', 'auto.png')
        self.psi.set_operation_mode(OperationModes.AUTO, Mock_qpixmap)
        Mock_qpixmap.assert_called_with(icon_path_auto)
        self.psi.labelOM.setPixmap.assert_called_with(ANY)
        self.psi.labelOM_text.setText.assert_called_with("Auto")

        # Operation mode T1 should load the correct icon and display the
        # correct text.
        icon_path_t1 = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', 't1.png')
        self.psi.set_operation_mode(OperationModes.T1, Mock_qpixmap)
        Mock_qpixmap.assert_called_with(icon_path_t1)
        self.psi.labelOM.setPixmap.assert_called_with(ANY)
        self.psi.labelOM_text.setText.assert_called_with("T1")

        # Operation mode T2 should load the correct icon and display the
        # correct text.
        icon_path_t2 = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', 't2.png')
        self.psi.set_operation_mode(OperationModes.T2, Mock_qpixmap)
        Mock_qpixmap.assert_called_with(icon_path_t2)
        self.psi.labelOM.setPixmap.assert_called_with(ANY)
        self.psi.labelOM_text.setText.assert_called_with("T2")

        # Operation mode Unknown should load the correct icon and display the
        # correct text.
        icon_path_unknown = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', 'unknown.png')
        self.psi.set_operation_mode(OperationModes.UNKNOWN, Mock_qpixmap)
        Mock_qpixmap.assert_called_with(icon_path_unknown)
        self.psi.labelOM.setPixmap.assert_called_with(ANY)
        self.psi.labelOM_text.setText.assert_called_with("Unknown")

    def test_set_speed(self):
        """Testing whether the speed override values are interpreted in the
        widget correctly, both for the conversion to percentage and the
        capping of input values between 0 and 1."""
        # Preparing mocks within the widget
        self.psi.barSpeed = MagicMock()
        self.psi.barSpeed.setValue = MagicMock()

        # 0 speed should be displayed as 0%
        self.psi.set_speed(0)
        self.psi.barSpeed.setValue.assert_called_with(0)

        # 0.5 speed should be displayed as 50%
        self.psi.set_speed(.5)
        self.psi.barSpeed.setValue.assert_called_with(50)

        # 1 speed should be displayed as 100%
        self.psi.set_speed(1)
        self.psi.barSpeed.setValue.assert_called_with(100)

        # speed values below 0 should be displayed as 0%
        self.psi.set_speed(-1)
        self.psi.barSpeed.setValue.assert_called_with(0)

        # speed values above 1 should be displayed as 100%
        self.psi.set_speed(2)
        self.psi.barSpeed.setValue.assert_called_with(100)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('pilz_status_indicator_rqt',
                    'test_status_indicator_widget', TestStatusIndicatorWidget)
