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

import rospy
from pilz_status_indicator_rqt.status_indicator_widget import \
    PilzStatusIndicatorWidget
from pilz_msgs.msg import OperationModes
from qt_gui.plugin import Plugin
from std_msgs.msg import Bool, Float64

TOPIC_OPERATION_MODE = "/prbt/operation_mode"
TOPIC_STATE_HW = "/prbt/status_info/state_hw"
TOPIC_STATE_ROS = "/prbt/status_info/state_ros"
TOPIC_SPEED_OVERRIDE = "/prbt/speed_override"


class PilzStatusIndicatorRqt(Plugin):
    """Implementation of the status indicator as rqt plugin. It will
    instantiate the Qt widget and handle the connection between ROS and its
    visual state."""

    def __init__(self, context):
        """Instantiate the plugin within its context with its initial state."""
        super(PilzStatusIndicatorRqt, self).__init__(context)
        self.setObjectName('PilzStatusIndicatorRqt')
        self._widget = PilzStatusIndicatorWidget(context.serial_number())

        # Hide currently unsupported elements
        self._widget.labelHW.hide()
        self._widget.labelHW_.hide()
        self._widget.labelROS.hide()
        self._widget.labelROS_.hide()
        self._widget.barSpeed.hide()

        # Set initial widget state
        self._widget.set_ROS_status(False)
        self._widget.set_HW_status(False)

        self._widget.set_operation_mode(OperationModes.UNKNOWN)

        self._widget.set_speed(.5)

        # Subscribing to all informations we want to display
        rospy.Subscriber(TOPIC_STATE_ROS, Bool, self.ros_status_callback)
        rospy.Subscriber(TOPIC_STATE_HW, Bool,
                         self.hw_status_callback)
        rospy.Subscriber(TOPIC_OPERATION_MODE, OperationModes,
                         self.operation_mode_callback)
        rospy.Subscriber(TOPIC_SPEED_OVERRIDE, Float64, self.speed_callback)

        context.add_widget(self._widget)

    def hw_status_callback(self, msg):
        """Callback for HW Status."""
        self._widget.set_HW_status(msg.data)

    def ros_status_callback(self, msg):
        """Callback for ROS Status."""
        self._widget.set_ROS_status(msg.data)

    def operation_mode_callback(self, msg):
        """Callback for Operation Mode."""
        rospy.logdebug("set_operation_mode: " + str(msg))
        self._widget.set_operation_mode(msg.value)

    def speed_callback(self, msg):
        """Callback for Speed Override."""
        val = msg.data
        self._widget.set_speed(val)
