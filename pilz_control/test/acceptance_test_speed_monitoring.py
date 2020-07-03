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

from std_msgs.msg import Float64

from controller_state_observer import ControllerStateObserver
from holding_mode_service_wrapper import HoldingModeServiceWrapper
from trajectory_dispatcher import TrajectoryDispatcher

_DEFAULT_CONTROLLER_NS = '/prbt'
_DEFAULT_CONTROLLER_NAME = 'manipulator_joint_trajectory_controller'

_CONTROLLER_NS_PARAMETER = '/controller_ns_string'
_CONTROLLER_NAME_PARAMTER = '/controller_name_string'
_JOINT_NAMES_PARAMETER = '/joint_names'
_MAX_FRAME_SPEED_TOPIC_NAME = '/max_frame_speed'

_SPEED_LIMIT = 0.25
_SPEED_LIMIT_TOLERANCE = 0.02
_VEL_SCALE_DEFAULT = 0.5
_LONG_TRAJ_CMD_DURATION = 10.0
_FRAME_SPEED_TOLERANCE = 0.001
_WAIT_FOR_CMD_FINISH_TIMEOUT = 3
_SLEEP_TIME = 3.0

_TEST_JOINT_START_POSITION = -0.5
_TEST_JOINT_MID_POSITION = 0.0
_TEST_JOINT_END_POSITION = 0.5
_TEST_JOINT_INDEX = 1
_TEST_JOINT_LOW_SPEED = 0.2
_TEST_JOINT_SPEED_LIMIT = 1.5


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
    """ Check if the controller successfully prevents a speed limit violation of the cartesian speed of the robot links.
        Prerequisites: Launch robot and joint_states_speed_observer.
    """

    def setUp(self):
        controller_ns = rospy.get_param(_CONTROLLER_NS_PARAMETER, _DEFAULT_CONTROLLER_NS)
        controller_name = rospy.get_param(_CONTROLLER_NAME_PARAMTER, _DEFAULT_CONTROLLER_NAME)

        param_name = controller_ns + _JOINT_NAMES_PARAMETER
        self.assertTrue(rospy.has_param(param_name))
        self._joint_names = rospy.get_param(param_name)

        self.assertTrue(len(self._joint_names) > _TEST_JOINT_INDEX)

        self._max_frame_speed = MaxFrameSpeedWrapper()
        # The observer can be used to ensure that trajectory goals are reached in order to have a clean test setup.
        self._robot_observer = ControllerStateObserver(controller_ns, controller_name)
        self._trajectory_dispatcher = TrajectoryDispatcher(controller_ns, controller_name)
        self._holding_mode_srv = HoldingModeServiceWrapper(controller_ns, controller_name)

        self._move_to_start_position()
        self._max_frame_speed.reset()

    def _unhold_controller(self):
        if not self._holding_mode_srv.request_default_mode():
            rospy.sleep(_SLEEP_TIME)
            self.assertTrue(self._holding_mode_srv.request_default_mode(), 'Unable to unhold controller')

    def _move_to_start_position(self):
        rospy.loginfo('Move to start position')
        self._unhold_controller()
        start_position = [0.0] * len(self._joint_names)
        start_position[_TEST_JOINT_INDEX] = _TEST_JOINT_START_POSITION
        self._trajectory_dispatcher.dispatch_single_point_continuous_trajectory(start_position)
        self._trajectory_dispatcher.wait_for_result()
        self._robot_observer.wait_for_position_reached(start_position)

    def _perform_test_movement(self, test_joint_velocity):
        self._unhold_controller()
        self._trajectory_dispatcher.dispatch_dual_point_trajectory(joint_index=_TEST_JOINT_INDEX,
                                                                   start_position=_TEST_JOINT_START_POSITION,
                                                                   mid_position=_TEST_JOINT_MID_POSITION,
                                                                   end_position=_TEST_JOINT_END_POSITION,
                                                                   velocity=test_joint_velocity)
        self._trajectory_dispatcher.wait_for_result()
        end_position = [0.0] * len(self._joint_names)
        end_position[_TEST_JOINT_INDEX] = _TEST_JOINT_END_POSITION
        self._robot_observer.wait_for_position_reached(end_position)

    def _compute_target_velocity(self):
        """ The target velocity (representing the speed limit) is computed via a formula and one speed observation.
            At the end of this function the robot moves to the start position.
        """
        rospy.loginfo('Determine target velocity for reaching the speed limit')
        self._max_frame_speed.reset(smooth_factor=0.1)
        self._perform_test_movement(_TEST_JOINT_LOW_SPEED)
        low_frame_speed = self._max_frame_speed.get()

        self._move_to_start_position()
        self._max_frame_speed.reset()
        return _TEST_JOINT_LOW_SPEED / low_frame_speed * _SPEED_LIMIT

    def test_reduced_speed_mode(self):
        """ Perform three movements with different velocities. One slightly below the speed limit, one slightly above
            and one movement with maximal joint speed. In any case the cartesian speed limit should not be violated,
            since we are in T1 mode.
        """
        rospy.loginfo('Test speed monitoring in T1 mode')

        target_velocity = self._compute_target_velocity()
        speed_limit_upper_bound = _SPEED_LIMIT + _SPEED_LIMIT_TOLERANCE

        self._perform_test_movement(0.9 * target_velocity)
        self.assertGreater(speed_limit_upper_bound, self._max_frame_speed.get(),
                           'Speed limit of 0.25[m/s] was violated')

        self._move_to_start_position()
        self._max_frame_speed.reset()

        self._perform_test_movement(1.1 * target_velocity)
        self.assertGreater(speed_limit_upper_bound, self._max_frame_speed.get(),
                           'Speed limit of 0.25[m/s] was violated. The limit might be too sharp. Did the robot' +
                           ' perform a successful stop?')

        self._move_to_start_position()
        self._max_frame_speed.reset()

        self._perform_test_movement(_TEST_JOINT_SPEED_LIMIT)
        self.assertGreater(speed_limit_upper_bound, self._max_frame_speed.get(),
                           'Speed limit of 0.25[m/s] was violated. The limit might be too sharp. Did the robot' +
                           ' perform a successful stop?')

    def test_automatic_mode(self):
        """ Perform a movement of the second joint with maximal joint speed. This results in a movement above
            the cartesian speed limit. Since we are in AUTO mode, the movement should not be interrupted.
        """
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
