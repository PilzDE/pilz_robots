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

import unittest
from mock import patch, ANY, Mock, MagicMock

import rospy

from python_qt_binding.QtCore import QObject
from qt_gui.plugin import Plugin

from pilz_status_indicator_rqt.status_indicator import PilzStatusIndicatorRqt, ISOViewWidget
from prbt_hardware_support.msg import OperationModes


class MockContext(QObject):
    def __init__(self):
        super(MockContext, self).__init__()
    def serial_number(self):
        return 0
    def add_widget(arg1, arg2):
        pass

class TestStatusIndicator(unittest.TestCase):
    """
    TODO
    """

    @patch('pilz_status_indicator_rqt.status_indicator.ISOViewWidget')
    def test_init(self, ViewMock):
        psi = PilzStatusIndicatorRqt(MockContext())
        psi._widget.set_ROS_status.assert_called_once_with(False)
        psi._widget.set_PRBT_status.assert_called_once_with(False)
        psi._widget.set_operation_mode.assert_called_once_with(OperationModes.UNKNOWN)
        psi._widget.set_speed.assert_called_once_with(0.5)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('pilz_status_indicator_rqt', 'test_status_indicator', TestStatusIndicator)
