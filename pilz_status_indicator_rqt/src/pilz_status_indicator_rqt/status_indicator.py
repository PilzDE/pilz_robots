import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from qt_gui.plugin_context import PluginContext
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QIcon, QColor, QPixmap
from python_qt_binding.QtWidgets import QWidget
from prbt_hardware_support.msg import OperationModes
from std_msgs.msg import Bool, Float64

from pilz_status_indicator_rqt import status_indicator_view

GREEN = "green"
RED = "red"

TOPIC_OPERATION_MODE = "/prbt/operation_mode"
TOPIC_DIAGNOSTICS_PRBT = "/prbt/diagnostics/state_prbt"
TOPIC_DIAGNOSTICS_ROS = "/prbt/diagnostics/state_ros"
TOPIC_SPEED_OVERRIDE = "/prbt/speed_override"


class PilzStatusIndicatorRqt(Plugin):
    def __init__(self, context):
        if context.__class__ == PluginContext:
            super(PilzStatusIndicatorRqt, self).__init__(context)
            self.setObjectName('PilzStatusIndicatorRqt')
            self._widget = QWidget()
            self.run(self._widget)
            # Show _widget.windowTitle on left-top of each plugin (when
            # it's set in _widget). This is useful when you open multiple
            # plugins at once.
            if context.serial_number() > 1:
                self._widget.setWindowTitle(
                    self._widget.windowTitle() + (' (%d)' % context.serial_number()))
            context.add_widget(self._widget)

    def run(self, widget):
        ui_file = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', 'PilzStatusIndicatorRqt.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('PilzStatusIndicatorRqtUi')

        # checking if widget is loaded correctly from ui file
        assert self._widget.labelROS, "ROS label must be loaded from ui file"
        assert self._widget.labelPRBT, "PRBT label must be loaded from ui file"
        assert self._widget.labelOM, "OM label must be loaded from ui file"
        assert self._widget.barSpeed, "barSpeed must be loaded from ui file"

        # prepare ui elements
        self._widget.labelOM.setScaledContents(True)

        # set intial state
        self._set_ROS_status_view(False)
        self._set_PRBT_status_view(False)
        self._set_operation_mode_view(OperationModes.UNKNOWN)
        self._set_speed_view(.5)

        rospy.Subscriber(TOPIC_DIAGNOSTICS_ROS, Bool,
                         self.ros_status_callback)
        rospy.Subscriber(TOPIC_DIAGNOSTICS_PRBT, Bool,
                         self.prbt_status_callback)
        rospy.Subscriber(TOPIC_OPERATION_MODE, OperationModes,
                         self.operation_mode_callback)
        rospy.Subscriber(TOPIC_SPEED_OVERRIDE, Float64, self.speed_callback)

    def shutdown_plugin(self):
        pass


    def prbt_status_callback(self, msg):
        self._set_PRBT_status_view(msg.data)

    def ros_status_callback(self, msg):
        self._set_ROS_status_view(msg.data)

    def _set_ROS_status_view(self, status):
        self._set_label_status_view(self._widget.labelROS, status)

    def _set_PRBT_status_view(self, status):
        self._set_label_status_view(self._widget.labelPRBT, status)

    def _set_label_status_view(self, label, status):
        if status:
            label.setStyleSheet("QLabel { background-color: %s }" % GREEN)
        else:
            label.setStyleSheet("QLabel { background-color: %s }" % RED)


    def operation_mode_callback(self, msg):
        rospy.logdebug("set_operation_mode: " + str(msg))
        self._set_operation_mode_view(msg.value)

    def _set_operation_mode_view(self, value):
        if value == OperationModes.AUTO:
            icon_name = 'auto'
        elif value == OperationModes.T1:
            icon_name = 't1'
        elif value == OperationModes.T2:
            icon_name = 't2'
        else:  # value == OperationModes.UNKNOWN
            icon_name = 'unknown'
        icon_path = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', icon_name + '.png')
        pixmap = QPixmap(icon_path)
        self._widget.labelOM.setPixmap(pixmap)


    def speed_callback(self, msg):
        val = msg.data
        self._set_speed_view(val)

    def _set_speed_view(self, val):
        if val > 1 or val < 0:  # expecting val = 0...1
            rospy.logwarn(
                "expecting speed value between 0 and 1, got {}".format(val))
            if val > 1:
                val = 1
            else:
                val = 0
        self._widget.barSpeed.setValue(100. * val)
