#!/usr/bin/env python
# Copyright (c) 2019 Pilz GmbH & Co. KG
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
from std_msgs.msg import Float64

from test_utils import HoldingModeServiceWrapper
from trajectory_dispatcher import TrajectoryDispatcher

# Change the following two lines if you run a different robot
CONTROLLER_NS = '/prbt'
CONTROLLER_NAME = 'manipulator_joint_trajectory_controller'

_JOINT_NAMES_PARAMETER = '/joint_names'
_STATE_TOPIC_NAME = '/state'
_MAX_FRAME_SPEED_TOPIC_NAME = '/max_frame_speed'

_SPEED_LIMIT = 0.25
_VEL_SCALE_DEFAULT = 0.5
_LONG_TRAJ_CMD_DURATION = 10.0
_SLEEP_RATE_HZ = 10
_POSITION_TOLERANCE = 0.01
_FRAME_SPEED_TOLERANCE = 0.001
_WAIT_FOR_CMD_FINISH_TIMEOUT = 3
_SLEEP_TIME = 3.0

_TEST_JOINT_START_POSITION = -0.5
_TEST_JOINT_MID_POSITION = 0.0
_TEST_JOINT_END_POSITION = 0.5
_TEST_JOINT_INDEX = 1
_TEST_JOINT_LOW_SPEED = 0.2
_TEST_JOINT_SPEED_LIMIT = 1.5

_WAIT_FOR_MESSAGE_TIMEOUT_S = 10

class SingleJointPositionObserver:

    def __init__(self, joint_index):
        self._joint_index = joint_index
        topic_name = CONTROLLER_NS + "/" + CONTROLLER_NAME + _STATE_TOPIC_NAME
        self._controller_state_sub = rospy.Subscriber(topic_name, JointTrajectoryControllerState, self._state_callback)
        self._actual_position = None
        self._actual_position_lock = threading.Lock()

        rospy.wait_for_message(topic_name, JointTrajectoryControllerState, _WAIT_FOR_MESSAGE_TIMEOUT_S)

    def _state_callback(self, msg):
        with self._actual_position_lock:
            self._actual_position = msg.actual.positions[self._joint_index]

    def wait_for_position_reached(self, position):
        rate = rospy.Rate(_SLEEP_RATE_HZ)
        start_waiting = rospy.Time.now()

        while True:
            with self._actual_position_lock:
                position_diff = position - self._actual_position

            if abs(position_diff) < _POSITION_TOLERANCE:
                return True

            if ((rospy.Time.now() - start_waiting).to_sec() > _WAIT_FOR_CMD_FINISH_TIMEOUT):
                return False

            rate.sleep()


class MaxFrameSpeedWrapper():

    def __init__(self, smooth_factor=1.0):
        self._smooth_factor = smooth_factor
        self._max_frame_speed = 0.0
        self._max_frame_speed_lock = threading.Lock()
        self._max_frame_speed_sub = rospy.Subscriber(_MAX_FRAME_SPEED_TOPIC_NAME, Float64,
                                                     self._max_frame_speed_callback)

    def get(self):
        with self._max_frame_speed_lock:
            return self._max_frame_speed

    def reset(self, smooth_factor=1.0):
        with self._max_frame_speed_lock:
            self._smooth_factor = smooth_factor
            self._max_frame_speed = 0.0

    def _max_frame_speed_callback(self, msg):
        """ Detects the maximum speed of all monitored links
        """
        with self._max_frame_speed_lock:
            smoothed_max_frame_speed = self._smooth_factor * msg.data + (1-self._smooth_factor) * self._max_frame_speed
            self._max_frame_speed = max(self._max_frame_speed, smoothed_max_frame_speed)


class AcceptancetestSpeedMonitoring(unittest.TestCase):
    """ Prerequisites: Launch robot and joint_states_speed_observer.
    """

    def setUp(self):
        param_name = CONTROLLER_NS + _JOINT_NAMES_PARAMETER
        self.assertTrue(rospy.has_param(param_name))
        self._joint_names = rospy.get_param(param_name)

        self.assertTrue(len(self._joint_names) > _TEST_JOINT_INDEX)

        self._max_frame_speed = MaxFrameSpeedWrapper()
        # The observer can be used to ensure that trajectory goals are reached in order to have a clean test setup.
        self._robot_observer = SingleJointPositionObserver(_TEST_JOINT_INDEX)
        self._trajectory_dispatcher = TrajectoryDispatcher(CONTROLLER_NS, CONTROLLER_NAME)
        self._holding_mode_srv = HoldingModeServiceWrapper(CONTROLLER_NS, CONTROLLER_NAME)

        self._move_to_start_position()
        self._max_frame_speed.reset()

    def _unhold_controller(self):
        if not self._holding_mode_srv.request_default_mode():
            rospy.sleep(_SLEEP_TIME)
            self.assertTrue(self._holding_mode_srv.request_default_mode(), 'Unable to unhold controller')

    def _move_to_start_position(self):
        rospy.loginfo('Move to start position')
        self._unhold_controller()
        self._trajectory_dispatcher.dispatch_single_point_continuous_trajectory(_TEST_JOINT_START_POSITION)
        self._trajectory_dispatcher.wait_for_result()
        self._robot_observer.wait_for_position_reached(_TEST_JOINT_START_POSITION)

    def _perform_test_movement(self, test_joint_velocity):
        self._unhold_controller()
        self._trajectory_dispatcher.dispatch_dual_point_trajectory(joint_index=_TEST_JOINT_INDEX,
                                                                   start_position=_TEST_JOINT_START_POSITION,
                                                                   mid_position=_TEST_JOINT_MID_POSITION,
                                                                   end_position=_TEST_JOINT_END_POSITION,
                                                                   velocity=test_joint_velocity)
        self._trajectory_dispatcher.wait_for_result()
        self._robot_observer.wait_for_position_reached(_TEST_JOINT_END_POSITION)

    def _compute_target_velocity(self):
        """ The target velocity (representing the speed limit) is computed via a formula and one speed observation.
            At the end of this function the robot moves to the start position.
        """
        rospy.loginfo('Determine target velocity for reaching the speed limit')
        self._max_frame_speed.reset(smooth_factor=0.5)
        self._perform_test_movement(_TEST_JOINT_LOW_SPEED)
        low_frame_speed = self._max_frame_speed.get()

        self._move_to_start_position()
        self._max_frame_speed.reset()
        return _TEST_JOINT_LOW_SPEED / low_frame_speed * _SPEED_LIMIT

    def test_reduced_speed_mode(self):
        rospy.loginfo('Test speed monitoring in T1 mode')

        target_velocity = self._compute_target_velocity()

        self._perform_test_movement(0.9 * target_velocity)
        self.assertGreater(_SPEED_LIMIT, self._max_frame_speed.get(), 'Speed limit of 0.25[m/s] was violated')

        self._move_to_start_position()
        self._max_frame_speed.reset()

        self._perform_test_movement(1.1 * target_velocity)
        self.assertGreater(_SPEED_LIMIT, self._max_frame_speed.get(), 'Speed limit of 0.25[m/s] was violated')

        self._move_to_start_position()
        self._max_frame_speed.reset()

        self._perform_test_movement(_TEST_JOINT_SPEED_LIMIT)
        self.assertGreater(_SPEED_LIMIT, self._max_frame_speed.get(), 'Speed limit of 0.25[m/s] was violated')

    def test_automatic_mode(self):
        rospy.loginfo('Test speed monitoring in AUTO mode')

        self._perform_test_movement(_TEST_JOINT_SPEED_LIMIT)
        self.assertLess(_SPEED_LIMIT, self._max_frame_speed.get(),
                        'Did not exceed speed limit of 0.25[m/s] as excepted')


if __name__ == "__main__":
    import rosunit
    import sys
    rospy.init_node('acceptance_test_speed_monitoring')
    if (len(sys.argv) > 1) and (sys.argv[1] == 'auto'):
        rosunit.unitrun('pilz_control', 'acceptance_test_speed_monitoring',
                        'acceptance_test_speed_monitoring.AcceptancetestSpeedMonitoring.test_automatic_mode')
    else:
        rosunit.unitrun('pilz_control', 'acceptance_test_speed_monitoring',
                        'acceptance_test_speed_monitoring.AcceptancetestSpeedMonitoring.test_reduced_speed_mode')
