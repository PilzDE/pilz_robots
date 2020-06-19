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

from controller_state_observer import ControllerStateObserver
from holding_mode_service_wrapper import HoldingModeServiceWrapper
from trajectory_dispatcher import TrajectoryDispatcher

# Change the following two lines if you run a different robot
CONTROLLER_NS = '/prbt'
CONTROLLER_NAME = 'manipulator_joint_trajectory_controller'

JOINT_NAMES_PARAMETER = '/joint_names'

TEST_JOINT_INDEX = 1
TEST_JOINT_START_POSITION = -0.5
TEST_JOINT_TARGET_POSITION = 0.5
TEST_JOINT_ACC_LIMIT = 3.49

SLEEP_UNHOLD_FAILURE_S = 3


class AcceptancetestAccelerationLimit(unittest.TestCase):

    def setUp(self):
        param_name = CONTROLLER_NS + JOINT_NAMES_PARAMETER
        self.assertTrue(rospy.has_param(param_name))
        self._joint_names = rospy.get_param(param_name)

        self._trajectory_dispatcher = TrajectoryDispatcher(CONTROLLER_NS, CONTROLLER_NAME)
        self._holding_mode_srv = HoldingModeServiceWrapper(CONTROLLER_NS, CONTROLLER_NAME)
        self._robot_observer = ControllerStateObserver(CONTROLLER_NS, CONTROLLER_NAME)

        self._start_position = [0.0]*len(self._joint_names)
        self._start_position[TEST_JOINT_INDEX] = TEST_JOINT_START_POSITION
        self._target_position = [0.0]*len(self._joint_names)
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

        actual_positions = self._robot_observer.get_actual_position()
        self.assertAlmostEqual(actual_positions[TEST_JOINT_INDEX], TEST_JOINT_START_POSITION,
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

        max_accelerations = self._robot_observer.get_max_acceleration()
        self.assertGreater(TEST_JOINT_ACC_LIMIT, max_accelerations[TEST_JOINT_INDEX],
                           'Acceleration limit was violated')


if __name__ == "__main__":
    import rostest

    rospy.init_node('acceptance_test_stop_trajectory')
    rostest.rosrun('pilz_control', 'acceptance_test_stop_trajectory', AcceptancetestAccelerationLimit)