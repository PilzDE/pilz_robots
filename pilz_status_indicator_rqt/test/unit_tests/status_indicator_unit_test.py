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

import rospy
from mock import ANY, MagicMock, Mock, call, patch
from pilz_msgs.msg import OperationModes
from python_qt_binding.QtCore import QObject
from qt_gui.plugin import Plugin
from std_msgs.msg import Bool, Float64

from pilz_status_indicator_rqt.status_indicator import *


class MockContext(QObject):
    def __init__(self):
        super(MockContext, self).__init__()

    def serial_number(self):
        return 0

    def add_widget(arg1, arg2):
        pass


def extract_subscriber_mock_callback(mock, topic_name):
    return [cb[1][2] for cb in mock.mock_calls if cb[1][0] == topic_name][0]


@patch('rospy.Subscriber')
@patch('pilz_status_indicator_rqt.status_indicator.PilzStatusIndicatorWidget')
class TestStatusIndicator(unittest.TestCase):
    """
    Testing the status indicator plugin to correctly interface with ROS.
    """

    def test_init(self, _, SubscriberMock):
        """Initialized Plugin and test if initialization methods are called
        correctly."""
        psi = PilzStatusIndicatorRqt(MockContext())
        psi._widget.set_ROS_status.assert_called_once_with(False)
        psi._widget.set_HW_status.assert_called_once_with(False)
        psi._widget.set_operation_mode.assert_called_once_with(
            OperationModes.UNKNOWN)
        psi._widget.set_speed.assert_called_once_with(0.5)
        SubscriberMock.assert_has_calls([
            call(TOPIC_STATE_ROS, Bool, ANY),
            call(TOPIC_STATE_HW, Bool, ANY),
            call(TOPIC_OPERATION_MODE, OperationModes, ANY),
            call(TOPIC_SPEED_OVERRIDE, Float64, ANY),
        ])

    def test_status_prbt(self, _, SubscriberMock):
        """Test if updates to the HW status are correctly passed from ROS to
        the widget."""
        psi = PilzStatusIndicatorRqt(MockContext())
        psi._widget.reset_mock()

        diagnostic_prbt_callback = extract_subscriber_mock_callback(
            SubscriberMock, TOPIC_STATE_HW)

        diagnostic_prbt_callback(Bool(False))
        psi._widget.set_HW_status.assert_called_with(False)

        diagnostic_prbt_callback(Bool(True))
        psi._widget.set_HW_status.assert_called_with(True)

    def test_status_ros(self, _, SubscriberMock):
        """Test if updates to the ROS status are correctly passed from ROS to
        the widget."""
        psi = PilzStatusIndicatorRqt(MockContext())
        psi._widget.reset_mock()

        diagnostic_ros_callback = extract_subscriber_mock_callback(
            SubscriberMock, TOPIC_STATE_ROS)

        diagnostic_ros_callback(Bool(True))
        psi._widget.set_ROS_status.assert_called_with(True)

        diagnostic_ros_callback(Bool(False))
        psi._widget.set_ROS_status.assert_called_with(False)

    def test_operation_mode(self, _, SubscriberMock):
        """Test if updates to the operation mode are correctly passed from ROS
        to the widget."""
        psi = PilzStatusIndicatorRqt(MockContext())
        psi._widget.reset_mock()

        diagnostic_operation_mode_callback = extract_subscriber_mock_callback(
            SubscriberMock, TOPIC_OPERATION_MODE)

        operation_mode_msg = OperationModes()

        operation_mode_msg.value = OperationModes.UNKNOWN
        diagnostic_operation_mode_callback(operation_mode_msg)
        psi._widget.set_operation_mode.assert_called_with(
            OperationModes.UNKNOWN)

        operation_mode_msg.value = OperationModes.T1
        diagnostic_operation_mode_callback(operation_mode_msg)
        psi._widget.set_operation_mode.assert_called_with(OperationModes.T1)

        operation_mode_msg.value = OperationModes.T2
        diagnostic_operation_mode_callback(operation_mode_msg)
        psi._widget.set_operation_mode.assert_called_with(OperationModes.T2)

        operation_mode_msg.value = OperationModes.AUTO
        diagnostic_operation_mode_callback(operation_mode_msg)
        psi._widget.set_operation_mode.assert_called_with(OperationModes.AUTO)

    def test_set_speed(self, _, SubscriberMock):
        """Test if updates to the speed override are correctly passed from ROS
        to the widget."""
        psi = PilzStatusIndicatorRqt(MockContext())
        psi._widget.reset_mock()

        speed_callback = extract_subscriber_mock_callback(
            SubscriberMock, TOPIC_SPEED_OVERRIDE)

        speed_msg = Float64()

        speed_msg.data = 0
        speed_callback(speed_msg)
        psi._widget.set_speed.assert_called_with(0)

        speed_msg.data = 0.5
        speed_callback(speed_msg)
        psi._widget.set_speed.assert_called_with(0.5)

        speed_msg.data = 1.0
        speed_callback(speed_msg)
        psi._widget.set_speed.assert_called_with(1.0)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('pilz_status_indicator_rqt',
                    'test_status_indicator', TestStatusIndicator)
