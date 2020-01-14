import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QIcon, QColor, QPixmap
from python_qt_binding.QtWidgets import QWidget
from prbt_hardware_support.msg import OperationModes
from std_msgs.msg import Bool, Float64

OK = "green"
NOK = "red"

TOPIC_OPERATION_MODE = "/prbt/operation_mode"
TOPIC_DIAGNOSTICS_PRBT = "/prbt/diagnostics/state_prbt"
TOPIC_DIAGNOSTICS_ROS = "/prbt/diagnostics/state_ros"
TOPIC_SPEED_OVERRIDE = "/prbt/speed_override"


class PilzStatusIndicatorRqt(Plugin):

    def __init__(self, context):
        super(PilzStatusIndicatorRqt, self).__init__(context)
        self.setObjectName('PilzStatusIndicatorRqt')
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', 'PilzStatusIndicatorRqt.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('PilzStatusIndicatorRqtUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # checking if widget is loaded correctly from ui file
        assert self._widget.labelROS, "ROS label must be loaded from ui file"
        assert self._widget.labelPRBT, "PRBT label must be loaded from ui file"
        assert self._widget.labelOM, "OM label must be loaded from ui file"
        assert self._widget.barSpeed, "barSpeed must be loaded from ui file"

        # prepare ui elements
        self._widget.labelROS.setStyleSheet(
            "QLabel { background-color: darkgrey }")
        self._widget.labelPRBT.setStyleSheet(
            "QLabel { background-color: darkgrey }")
        self._widget.labelOM.setScaledContents(True)

        # set intial state
        self.set_ROS_status(Bool(False))
        self.set_PRBT_status(Bool(False))
        om_unknown = OperationModes()
        om_unknown.value = OperationModes.UNKNOWN
        self.set_operation_mode(om_unknown)
        self.set_speed(Float64(.5))

        rospy.Subscriber(TOPIC_DIAGNOSTICS_ROS, Bool,
                         self.set_ROS_status)
        rospy.Subscriber(TOPIC_DIAGNOSTICS_PRBT, Bool,
                         self.set_PRBT_status)
        rospy.Subscriber(TOPIC_OPERATION_MODE, OperationModes,
                         self.set_operation_mode)
        rospy.Subscriber(TOPIC_SPEED_OVERRIDE, Float64, self.set_speed)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def set_ROS_status(self, msg):
        status = msg.data
        self._set_label_status(self._widget.labelROS, status)

    def set_PRBT_status(self, msg):
        status = msg.data
        self._set_label_status(self._widget.labelPRBT, status)

    def set_operation_mode(self, operation_mode):
        rospy.logdebug("set_operation_mode: " + str(operation_mode))
        if operation_mode.value == OperationModes.AUTO:
            icon_name = 'auto'
        elif operation_mode.value == OperationModes.T1:
            icon_name = 't1'
        elif operation_mode.value == OperationModes.T2:
            icon_name = 't2'
        else:  # operation_mode.value == OperationModes.UNKNOWN
            icon_name = 'unknown'
        icon_path = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', icon_name + '.png')
        pixmap = QPixmap(icon_path)
        self._widget.labelOM.setPixmap(pixmap)

    def set_speed(self, msg):
        val = msg.data
        # expecting val = 0...1
        if val > 1 or val < 0:
            rospy.logwarn("expecting speed value between 0 and 1, got {}".format(val))
            if val > 1:
                val = 1
            else:
                val = 0
        self._widget.barSpeed.setValue(100. * val)

    def _set_label_status(self, label, status):
        if status:
            label.setStyleSheet("QLabel { background-color: %s }" % OK)
        else:
            label.setStyleSheet("QLabel { background-color: %s }" % NOK)
