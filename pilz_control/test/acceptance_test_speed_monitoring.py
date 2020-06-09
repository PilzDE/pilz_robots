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

import actionlib
import math
import numpy
import rospy
import threading
import unittest

from control_msgs.msg import *
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger, TriggerRequest

_MAX_FRAME_SPEED_TOPIC_NAME = '/max_frame_speed'
_SPEED_LIMIT = 0.25
_VEL_SCALE_DEFAULT = 0.5
_LONG_TRAJ_CMD_DURATION = 10.0
_SLEEP_RATE_HZ = 10
_POSITION_TOLERANCE = 0.01
_WAIT_FOR_CMD_FINISH_TIMEOUT = 3

_START_POSITION = [0.0, -0.5, 0.0, 0.0, 0.0, 0.0]
_TARGET_POSITION = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]

_WAIT_FOR_SERVICE_TIMEOUT_S = 10
_WAIT_FOR_MESSAGE_TIMEOUT_S = 10
_JOINT_NAMES = ['prbt_joint_1', 'prbt_joint_2', 'prbt_joint_3', 'prbt_joint_4', 'prbt_joint_5', 'prbt_joint_6']

_UNHOLD_SERVICE_NAME = '/prbt/manipulator_joint_trajectory_controller/unhold'
_FOLLOW_JOINT_TRAJ_ACTION_NAME = '/prbt/manipulator_joint_trajectory_controller/follow_joint_trajectory'
_STATE_TOPIC_NAME = '/prbt/manipulator_joint_trajectory_controller/state'


class SinglePointTrajectoryDispatcher:

    def __init__(self):
        self._client = actionlib.SimpleActionClient(_FOLLOW_JOINT_TRAJ_ACTION_NAME, FollowJointTrajectoryAction)

        timeout = rospy.Duration(_WAIT_FOR_SERVICE_TIMEOUT_S)
        self._client.wait_for_server(timeout)

    def send_action_goal(self, position=[0.0]*len(_JOINT_NAMES), velocity=[], velocity_tolerance=0.0,
                         time_from_start=_LONG_TRAJ_CMD_DURATION):
        assert len(position) == len(_JOINT_NAMES)
        if velocity:
            assert len(velocity) == len(_JOINT_NAMES)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = _JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = position
        point.velocities = velocity
        point.time_from_start = rospy.Duration(time_from_start)

        goal.trajectory.points = [point]

        for joint_name in _JOINT_NAMES:
            tol = JointTolerance()
            tol.name = joint_name
            tol.velocity = velocity_tolerance
            goal.path_tolerance.append(tol)

        self._client.send_goal_and_wait(goal)


class RobotPositionObserver:

    def __init__(self):
        self._controller_state_sub = rospy.Subscriber(_STATE_TOPIC_NAME, JointTrajectoryControllerState, self._state_callback)
        self._actual_position = None
        self._actual_position_lock = threading.Lock()

        rospy.wait_for_message(_STATE_TOPIC_NAME, JointTrajectoryControllerState, _WAIT_FOR_MESSAGE_TIMEOUT_S)

    def _state_callback(self, msg):
        with self._actual_position_lock:
            self._actual_position = msg.actual.positions

    def wait_for_position_reached(self, position):
        assert len(position) == len(_JOINT_NAMES)

        rate = rospy.Rate(_SLEEP_RATE_HZ)
        start_waiting = rospy.Time.now()

        while True:
            with self._actual_position_lock:
                position_diff = numpy.array(position) - numpy.array(self._actual_position)

            if (numpy.linalg.norm(position_diff) < _POSITION_TOLERANCE):
                return True

            if ((rospy.Time.now() - start_waiting).to_sec() > _WAIT_FOR_CMD_FINISH_TIMEOUT):
                return False

            rate.sleep()


class UnholdServiceWrapper():

    def __init__(self):
        rospy.wait_for_service(_UNHOLD_SERVICE_NAME)
        self._unhold_service = rospy.ServiceProxy(_UNHOLD_SERVICE_NAME, Trigger)

    def call(self):
        req = TriggerRequest()
        self._unhold_service(req)


class MaxFrameSpeedWrapper():

    def __init__(self):
        self._max_frame_speed = 0.0
        self._max_frame_speed_lock = threading.Lock()
        self._max_frame_speed_sub = rospy.Subscriber(_MAX_FRAME_SPEED_TOPIC_NAME, Float64,
                                                     self._max_frame_speed_callback)

    def get(self):
        with self._max_frame_speed_lock:
            return self._max_frame_speed

    def reset(self):
        with self._max_frame_speed_lock:
            self._max_frame_speed = 0.0

    def _max_frame_speed_callback(self, msg):
        """ Detects the maximum speed of all monitored links
        """
        with self._max_frame_speed_lock:
            self._max_frame_speed = max(self._max_frame_speed, msg.data)


class AcceptancetestSpeedMonitoring(unittest.TestCase):
    """ Prerequisites: Launch robot and joint_states_speed_observer.
    """

    def setUp(self):
        self._max_frame_speed = MaxFrameSpeedWrapper()
        # The observer can be used to ensure that trajectory goals are reached in order to have a clean test setup.
        self._robot_observer = RobotPositionObserver()
        self._trajectory_dispatcher = SinglePointTrajectoryDispatcher()
        self._unhold_service = UnholdServiceWrapper()

        self._determine_target_duration()
        self._max_frame_speed.reset()

    def _determine_target_duration(self):
        """ The target duration (representing the speed limit) is computed via a formula and one speed observation.
            At the end of this function the robot reaches the start position.
        """
        self._unhold_service.call()
        self._trajectory_dispatcher.send_action_goal(position=_TARGET_POSITION)
        self._robot_observer.wait_for_position_reached(_TARGET_POSITION)

        self._max_frame_speed.reset()
        self._trajectory_dispatcher.send_action_goal(position=_START_POSITION)
        self._robot_observer.wait_for_position_reached(_START_POSITION)

        self._target_duration = _LONG_TRAJ_CMD_DURATION * self._max_frame_speed.get() / _SPEED_LIMIT

    def _perform_scaled_movement(self, duration_scale):
        """ Send a trajectory command where the target duration is multiplied by a scaling factor.
            Then move back to start position.
        """
        self._max_frame_speed.reset()
        time_from_start = duration_scale * self._target_duration
        self._trajectory_dispatcher.send_action_goal(position=_TARGET_POSITION, time_from_start=time_from_start)

        self._unhold_service.call()  # in case movement failed
        self._trajectory_dispatcher.send_action_goal(position=_START_POSITION, time_from_start=_LONG_TRAJ_CMD_DURATION)
        self._robot_observer.wait_for_position_reached(_START_POSITION)

    def test_reduced_speed_mode(self):

        self._perform_scaled_movement(1.1)
        self.assertGreater(_SPEED_LIMIT, self._max_frame_speed, 'Speed limit of 0.25[m/s] was violated')

        self._perform_scaled_movement(0.9)
        self.assertGreater(_SPEED_LIMIT, self._max_frame_speed, 'Speed limit of 0.25[m/s] was violated')

    def test_automatic_mode(self):

        self._perform_scaled_movement(1.1)
        self.assertLess(_SPEED_LIMIT, self._max_frame_speed, 'Did not exceed speed limit of 0.25[m/s] as excepted')


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
