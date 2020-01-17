import os

import rospy
from pilz_status_indicator_rqt.status_indicator_widget import \
    PilzStatusIndicatorWidget
from prbt_hardware_support.msg import OperationModes
from qt_gui.plugin import Plugin
from std_msgs.msg import Bool, Float64

TOPIC_OPERATION_MODE = "/prbt/operation_mode"
TOPIC_DIAGNOSTICS_PRBT = "/prbt/diagnostics/state_prbt"
TOPIC_DIAGNOSTICS_ROS = "/prbt/diagnostics/state_ros"
TOPIC_SPEED_OVERRIDE = "/prbt/speed_override"


class PilzStatusIndicatorRqt(Plugin):
    def __init__(self, context):
        super(PilzStatusIndicatorRqt, self).__init__(context)
        self.setObjectName('PilzStatusIndicatorRqt')
        self._widget = PilzStatusIndicatorWidget()

        # set intial state
        self._widget.set_ROS_status(False)
        self._widget.set_PRBT_status(False)

        self._widget.set_operation_mode(OperationModes.UNKNOWN)

        self._widget.set_speed(.5)

        rospy.Subscriber(TOPIC_DIAGNOSTICS_ROS, Bool, self.ros_status_callback)
        rospy.Subscriber(TOPIC_DIAGNOSTICS_PRBT, Bool,
                         self.prbt_status_callback)
        rospy.Subscriber(TOPIC_OPERATION_MODE, OperationModes,
                         self.operation_mode_callback)
        rospy.Subscriber(TOPIC_SPEED_OVERRIDE, Float64, self.speed_callback)

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass

    def prbt_status_callback(self, msg):
        self._widget.set_PRBT_status(msg.data)

    def ros_status_callback(self, msg):
        self._widget.set_ROS_status(msg.data)

    def operation_mode_callback(self, msg):
        rospy.logdebug("set_operation_mode: " + str(msg))
        self._widget.set_operation_mode(msg.value)

    def speed_callback(self, msg):
        val = msg.data
        self._widget.set_speed(val)
