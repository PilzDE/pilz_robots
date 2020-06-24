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

import numpy
import rospy
import threading

from control_msgs.msg import JointTrajectoryControllerState


class ControllerStateObserver:

    STATE_TOPIC_NAME = 'state'
    WAIT_FOR_MESSAGE_TIMEOUT_S = 10
    SLEEP_RATE_HZ = 10
    POSITION_TOLERANCE = 0.01
    WAIT_FOR_CMD_FINISH_TIMEOUT_S = 3

    def __init__(self, controller_ns, controller_name):
        topic_name = controller_ns + "/" + controller_name + "/" + self.STATE_TOPIC_NAME
        self._controller_state_sub = rospy.Subscriber(topic_name, JointTrajectoryControllerState, self._state_callback)

        self._actual_position = None
        self._actual_velocity = None
        self._msg_time_stamp = []
        self._max_acceleration = []
        self._controller_state_lock = threading.Lock()

        rospy.wait_for_message(topic_name, JointTrajectoryControllerState, self.WAIT_FOR_MESSAGE_TIMEOUT_S)

    def _state_callback(self, msg):

        with self._controller_state_lock:
            self._actual_position = msg.actual.positions
            old_velocity = self._actual_velocity
            self._actual_velocity = msg.actual.velocities
            old_time_stamp = self._msg_time_stamp
            self._msg_time_stamp = msg.header.stamp

            # acceleration is not available, so we compute it here
            if old_time_stamp:
                delta_t = (self._msg_time_stamp - old_time_stamp).to_sec()
                abs_velocity = numpy.absolute(self._actual_velocity)
                abs_old_velocity = numpy.absolute(old_velocity)
                self._actual_acceleration = (numpy.array(abs_velocity) - numpy.array(abs_old_velocity)) / delta_t

                if self._max_acceleration:
                    # determine the maximal acceleration for each joint
                    self._max_acceleration = [max(a, b) for a, b in zip(self._max_acceleration,
                                                                        self._actual_acceleration)]
                else:
                    self._max_acceleration = list(self._actual_acceleration)

    def get_actual_position(self):
        """ Get the actual joint position of the robot."""
        with self._controller_state_lock:
            return self._actual_position

    def get_actual_velocity(self):
        """ Get the actual joint velocity of the robot."""
        with self._controller_state_lock:
            return self._actual_velocity

    def get_actual_acceleration(self):
        """ Get the actual joint acceleration of the robot."""
        with self._controller_state_lock:
            return self._actual_acceleration

    def get_max_acceleration(self):
        """ Get the maximal acceleration for each joint."""
        with self._controller_state_lock:
            return self._max_acceleration

    def reset_max_acceleration(self):
        with self._controller_state_lock:
            self._max_acceleration = [0.0] * len(self._max_acceleration)

    def wait_for_position_reached(self, position):
        rate = rospy.Rate(self.SLEEP_RATE_HZ)
        start_waiting = rospy.Time.now()

        while True:
            with self._controller_state_lock:
                position_diff = numpy.array(position) - numpy.array(self._actual_position)

            if numpy.linalg.norm(position_diff) < self.POSITION_TOLERANCE:
                return True

            if ((rospy.Time.now() - start_waiting).to_sec() > self.WAIT_FOR_CMD_FINISH_TIMEOUT_S):
                return False

            rate.sleep()
