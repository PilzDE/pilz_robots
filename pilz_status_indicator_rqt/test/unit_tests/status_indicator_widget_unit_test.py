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

import sys
import os
import unittest

import rospkg
import rospy
from mock import ANY, MagicMock, Mock, call, patch
from pilz_status_indicator_rqt.status_indicator_widget import (
    GREEN, RED, PilzStatusIndicatorWidget)
from python_qt_binding.QtWidgets import QApplication, QMainWindow
from prbt_hardware_support.msg import OperationModes

app = QApplication(sys.argv)


class TestStatusIndicatorWidget(unittest.TestCase):
    """
    TODO
    """

    def setUp(self):
        self.psi = PilzStatusIndicatorWidget()
        self.icon_path_auto = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', 'auto.png')
        self.icon_path_t1 = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', 't1.png')
        self.icon_path_t2 = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', 't2.png')
        self.icon_path_unknown = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', 'unknown.png')

    def test_set_ROS_status(self):
        self.psi.labelROS.setStyleSheet = MagicMock()
        self.psi.set_ROS_status(True)
        self.psi.labelROS.setStyleSheet.assert_called_with(
            "QLabel { background-color: %s }" % GREEN)

        self.psi.set_ROS_status(False)
        self.psi.labelROS.setStyleSheet.assert_called_with(
            "QLabel { background-color: %s }" % RED)

    def test_set_PRBT_status(self):
        self.psi.labelPRBT.setStyleSheet = MagicMock()
        self.psi.set_PRBT_status(True)
        self.psi.labelPRBT.setStyleSheet.assert_called_with(
            "QLabel { background-color: %s }" % GREEN)

        self.psi.set_PRBT_status(False)
        self.psi.labelPRBT.setStyleSheet.assert_called_with(
            "QLabel { background-color: %s }" % RED)

    @patch('rospkg.RosPack')
    def test_set_operation_mode_auto(self, RosPackMock):
        self.psi.labelOM.setPixmap = MagicMock()
        RosPackMock.get_path = MagicMock(
            return_value=self.icon_path_unknown)

        RosPackMock.assert_called_with(self.icon_path_auto)
        self.psi.set_operation_mode(OperationModes.AUTO)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('pilz_status_indicator_rqt',
                    'test_status_indicator_widget', TestStatusIndicatorWidget)
