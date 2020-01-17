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

GREEN = "green"
RED = "red"

TOPIC_OPERATION_MODE = "/prbt/operation_mode"
TOPIC_DIAGNOSTICS_PRBT = "/prbt/diagnostics/state_prbt"
TOPIC_DIAGNOSTICS_ROS = "/prbt/diagnostics/state_ros"
TOPIC_SPEED_OVERRIDE = "/prbt/speed_override"

class ISOViewWidget(QWidget):
    def __init__(self):
        super(ISOViewWidget, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path(
            'pilz_status_indicator_rqt'), 'resource', 'PilzStatusIndicatorRqt.ui')
        loadUi(ui_file, self)
        self.setObjectName('PilzStatusIndicatorRqtUi')

        # checking if widget is loaded correctly from ui file
        assert self.labelROS, "ROS label must be loaded from ui file"
        assert self.labelPRBT, "PRBT label must be loaded from ui file"
        assert self.labelOM, "OM label must be loaded from ui file"
        assert self.barSpeed, "barSpeed must be loaded from ui file"

        # prepare ui elements
        self.labelOM.setScaledContents(True)

    def _set_label_status_view(self, label, status):
        if status:
            label.setStyleSheet("QLabel { background-color: %s }" % GREEN)
        else:
            label.setStyleSheet("QLabel { background-color: %s }" % RED)

    def set_ROS_status(self, status):
        print("set ros status")
        self._set_label_status_view(self.labelROS, status)

    def set_PRBT_status(self, status):
        self._set_label_status_view(self.labelPRBT, status)

    def set_operation_mode(self, mode):
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
        pixmap = QPixmap(icon_path)
        self.labelOM.setPixmap(pixmap)

    def set_speed(self, val):
        if val > 1 or val < 0:  # expecting val = 0...1
            rospy.logwarn(
                "expecting speed value between 0 and 1, got {}".format(val))
            if val > 1:
                val = 1
            else:
                val = 0
        self.barSpeed.setValue(100. * val)



class PilzStatusIndicatorRqt(Plugin):
    def __init__(self, context):
        super(PilzStatusIndicatorRqt, self).__init__(context)
        self.setObjectName('PilzStatusIndicatorRqt')
        self._widget = ISOViewWidget()

        # set intial state
        self._widget.set_ROS_status(False)
        self._widget.set_PRBT_status(False)

        self._widget.set_operation_mode(OperationModes.UNKNOWN)

        self._widget.set_speed(.5)

        rospy.Subscriber(TOPIC_DIAGNOSTICS_ROS, Bool, self.ros_status_callback)
        rospy.Subscriber(TOPIC_DIAGNOSTICS_PRBT, Bool, self.prbt_status_callback)
        rospy.Subscriber(TOPIC_OPERATION_MODE, OperationModes, self.operation_mode_callback)
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
