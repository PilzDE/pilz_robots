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

import rospy
import threading
import unittest

from control_msgs.msg import JointTrajectoryControllerState

from test_utils import HoldingModeServiceWrapper
from trajectory_dispatcher import TrajectoryDispatcher

CONTROLLER_NS = '/prbt'
CONTROLLER_NAME = 'manipulator_joint_trajectory_controller'
CONTROLLER_STATE_TOPIC_NAME = '/prbt/manipulator_joint_trajectory_controller/state'
JOINT_NAMES = ['prbt_joint_1', 'prbt_joint_2', 'prbt_joint_3', 'prbt_joint_4', 'prbt_joint_5', 'prbt_joint_6']

TEST_JOINT_INDEX = 1
TEST_JOINT_START_POSITION = -0.5
TEST_JOINT_TARGET_POSITION = 0.5
TEST_JOINT_ACC_LIMIT = 3.49

DEFAULT_TRAJECTORY_DURATION_S = 10
WAIT_FOR_SERVICE_TIMEOUT_S = 10
WAIT_FOR_MESSAGE_TIMEOUT_S = 10


class SingleJointStateObserver:

    def __init__(self, joint_index):
        self._joint_index = joint_index

        topic_name = CONTROLLER_STATE_TOPIC_NAME
        self._controller_state_sub = rospy.Subscriber(topic_name, JointTrajectoryControllerState, self._state_callback)

        self._actual_position = None
        self._actual_position_lock = threading.Lock()
        self._actual_velocity = None
        self._actual_velocity_lock = threading.Lock()

        self._last_time_stamp = []
        self._max_acceleration = 0.0

        rospy.wait_for_message(topic_name, JointTrajectoryControllerState, WAIT_FOR_MESSAGE_TIMEOUT_S)

    def _state_callback(self, msg):
        time_delta = []
        if self._last_time_stamp:
            time_delta = msg.header.stamp - self._last_time_stamp

        with self._actual_position_lock:
            self._actual_position = msg.actual.positions[self._joint_index]

        with self._actual_velocity_lock:
            actual_velocity = msg.actual.velocities[self._joint_index]
            self._actual_velocity = actual_velocity
            if time_delta:
                actual_acceleration = (actual_velocity - self._actual_velocity) / time_delta
                self._max_acceleration = max(self._max_acceleration, actual_acceleration)

    def get_actual_position(self):
        with self._actual_position_lock:
            return self._actual_position

    def get_max_acceleration(self):
        with self._actual_velocity_lock:
            return self._max_acceleration

    def reset_max_acceleration(self):
        with self._actual_velocity_lock:
            self._max_acceleration = 0.0


class AcceptancetestAccelerationLimit(unittest.TestCase):

    def setUp(self):
        self._trajectory_dispatcher = TrajectoryDispatcher(CONTROLLER_NS, CONTROLLER_NAME)
        self._holding_mode_srv = HoldingModeServiceWrapper(CONTROLLER_NS, CONTROLLER_NAME)
        self._robot_observer = SingleJointStateObserver(TEST_JOINT_INDEX)

        self._start_position = [0.0]*len(JOINT_NAMES)
        self._start_position[TEST_JOINT_INDEX] = TEST_JOINT_START_POSITION
        self._target_position = [0.0]*len(JOINT_NAMES)
        self._target_position[TEST_JOINT_INDEX] = TEST_JOINT_TARGET_POSITION

        self._unhold_controller()

    def _unhold_controller(self):
        if not self._holding_mode_srv.request_default_mode():
            rospy.sleep(SLEEP_UNHOLD_FAILURE_S)
            self.assertTrue(self._holding_mode_srv.request_default_mode(), 'Unable to unhold controller')

    def test_critical_discontinuous_movement(self):
        rospy.loginfo('!!! BE CAREFUL. ROBOT MIGHT CRASH. !!!')
        rospy.sleep(3.0)

        rospy.loginfo('First move to start position...')
        self._trajectory_dispatcher.dispatch_single_point_continuous_trajectory(self._start_position)
        self._trajectory_dispatcher.wait_for_result()
        self._unhold_controller()
        self._trajectory_dispatcher.dispatch_single_point_trajectory(self._target_position, time_from_start=0.02)
        self._trajectory_dispatcher.wait_for_result()

        self.assertAlmostEqual(self._robot_observer.get_actual_position(), TEST_JOINT_START_POSITION,
                               msg='Robot did not stand still as expected', delta=0.01)

    def test_critical_continuous_movement(self):
        rospy.loginfo('!!! BE CAREFUL. ROBOT MIGHT CRASH. !!!')
        rospy.sleep(3.0)

        rospy.loginfo('First move to start position...')
        self._trajectory_dispatcher.dispatch_single_point_continuous_trajectory(self._start_position)
        self._trajectory_dispatcher.wait_for_result()
        self._unhold_controller()
        self._robot_observer.reset_max_acceleration()
        self._trajectory_dispatcher.dispatch_single_point_continuous_trajectory(self._target_position,
                                                                                time_from_start=0.02)
        self._trajectory_dispatcher.wait_for_result()

        self.assertGreater(TEST_JOINT_ACC_LIMIT, self._robot_observer.get_max_acceleration(),
                           'Acceleration limit was violated')


if __name__ == "__main__":
    import rostest

    rospy.init_node('acceptance_test_stop_trajectory')
    rostest.rosrun('pilz_control', 'acceptance_test_stop_trajectory', AcceptancetestAccelerationLimit)
