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

# Change the following two lines if you run a different robot
_PREFIX_CONTROLLER = '/prbt/manipulator_joint_trajectory_controller'
_PREFIX_CONTROLLER_NAMESPACE = '/prbt'

_JOINT_NAMES_PARAMETER = '/joint_names'
_UNHOLD_SERVICE_NAME = '/unhold'
_FOLLOW_JOINT_TRAJ_ACTION_NAME = '/follow_joint_trajectory'
_STATE_TOPIC_NAME = '/state'
_MAX_FRAME_SPEED_TOPIC_NAME = '/max_frame_speed'

_SPEED_LIMIT = 0.25
_VEL_SCALE_DEFAULT = 0.5
_LONG_TRAJ_CMD_DURATION = 10.0
_SLEEP_RATE_HZ = 10
_POSITION_TOLERANCE = 0.01
_WAIT_FOR_CMD_FINISH_TIMEOUT = 3
_SLEEP_TIME = 3.0

_JOINT2_START_POSITION = -0.5
_JOINT2_TARGET_POSITION = 0.5
_JOINT2_INDEX = 1
_JOINT2_SPEED_LIMIT = 1.5

_WAIT_FOR_SERVICE_TIMEOUT_S = 10
_WAIT_FOR_MESSAGE_TIMEOUT_S = 10


class SinglePointTrajectoryDispatcher:

    def __init__(self, default_joint_names):
        self._default_joint_names = default_joint_names

        action_name = _PREFIX_CONTROLLER + _FOLLOW_JOINT_TRAJ_ACTION_NAME
        self._client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)

        timeout = rospy.Duration(_WAIT_FOR_SERVICE_TIMEOUT_S)
        self._client.wait_for_server(timeout)

    def send_action_goal(self, joint_names=[], position=[], velocity=[], velocity_tolerance=0.0,
                         time_from_start=_LONG_TRAJ_CMD_DURATION):
        if not joint_names:
            joint_names = self._default_joint_names
        if not position:
            position = [0.0]*len(joint_names)

        assert len(position) == len(joint_names)
        if velocity:
            assert len(velocity) == len(joint_names)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = position
        point.velocities = velocity
        point.time_from_start = rospy.Duration(time_from_start)

        goal.trajectory.points = [point]

        for joint_name in joint_names:
            tol = JointTolerance()
            tol.name = joint_name
            tol.velocity = velocity_tolerance
            goal.path_tolerance.append(tol)

        self._client.send_goal_and_wait(goal)


class RobotPositionObserver:

    def __init__(self):
        topic_name = _PREFIX_CONTROLLER + _STATE_TOPIC_NAME
        self._controller_state_sub = rospy.Subscriber(topic_name, JointTrajectoryControllerState, self._state_callback)
        self._actual_position = None
        self._actual_position_lock = threading.Lock()

        rospy.wait_for_message(topic_name, JointTrajectoryControllerState, _WAIT_FOR_MESSAGE_TIMEOUT_S)

    def _state_callback(self, msg):
        with self._actual_position_lock:
            self._actual_position = msg.actual.positions

    def wait_for_position_reached(self, position):
        with self._actual_position_lock:
            assert len(position) == len(self._actual_position)

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
        service_name = _PREFIX_CONTROLLER + _UNHOLD_SERVICE_NAME
        rospy.wait_for_service(service_name)
        self._unhold_service = rospy.ServiceProxy(service_name, Trigger)

    def call(self):
        req = TriggerRequest()
        resp = self._unhold_service(req)
        return resp.success


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
        param_name = _PREFIX_CONTROLLER_NAMESPACE + _JOINT_NAMES_PARAMETER
        self.assertTrue(rospy.has_param(param_name))
        self._joint_names = rospy.get_param(param_name)

        self.assertTrue(len(self._joint_names) > _JOINT2_INDEX)

        self._start_position = [0.0]*len(self._joint_names)
        self._target_position = [0.0]*len(self._joint_names)
        self._start_position[_JOINT2_INDEX] = _JOINT2_START_POSITION
        self._target_position[_JOINT2_INDEX] = _JOINT2_TARGET_POSITION

        self._max_frame_speed = MaxFrameSpeedWrapper()
        # The observer can be used to ensure that trajectory goals are reached in order to have a clean test setup.
        self._robot_observer = RobotPositionObserver()
        self._trajectory_dispatcher = SinglePointTrajectoryDispatcher(self._joint_names)
        self._unhold_service = UnholdServiceWrapper()

        self._move_to_start_position()
        self._max_frame_speed.reset()

    def _move_to_start_position(self):
        rospy.loginfo('Move to start position')

        if not self._unhold_service.call():
            rospy.sleep(_SLEEP_TIME)
            self.assertTrue(self._unhold_service.call(), 'Unable to unhold controller')

        self._trajectory_dispatcher.send_action_goal(position=self._start_position)
        self._robot_observer.wait_for_position_reached(self._start_position)

    def _determine_target_duration(self):
        """ The target duration (representing the speed limit) is computed via a formula and one speed observation.
            At the end of this function the robot reaches the start position.
        """
        rospy.loginfo('Determine target duration for reaching the speed limit')
        self._trajectory_dispatcher.send_action_goal(position=self._target_position)
        self._robot_observer.wait_for_position_reached(self._target_position)

        self._target_duration = _LONG_TRAJ_CMD_DURATION * self._max_frame_speed.get() / _SPEED_LIMIT

        self._move_to_start_position()

    def _compute_min_duration(self):
        """ The minimal duration can be computed from the maximal velocity of joint2.
        """
        return (_JOINT2_TARGET_POSITION - _JOINT2_START_POSITION) / _JOINT2_SPEED_LIMIT

    def _perform_scaled_movement(self, duration_scale):
        """ Send a trajectory command where the target duration is multiplied by a scaling factor.
            Then move back to start position.
        """
        time_from_start = duration_scale * self._target_duration
        self._trajectory_dispatcher.send_action_goal(position=self._target_position, time_from_start=time_from_start)

        self._move_to_start_position()
        rospy.loginfo('Finished scaled movement')

    def test_reduced_speed_mode(self):
        rospy.loginfo('Test speed monitoring in T1 mode')

        self._determine_target_duration()

        self._max_frame_speed.reset()
        self._perform_scaled_movement(1.1)
        self.assertGreater(_SPEED_LIMIT, self._max_frame_speed.get(), 'Speed limit of 0.25[m/s] was violated')

        self._max_frame_speed.reset()
        self._perform_scaled_movement(0.9)
        self.assertGreater(_SPEED_LIMIT, self._max_frame_speed.get(), 'Speed limit of 0.25[m/s] was violated')

        rospy.loginfo('!!!BE CAREFUL, ROBOT MIGHT CRASH INTO TABLE!!!')
        rospy.sleep(_SLEEP_TIME)

        self._max_frame_speed.reset()
        self._trajectory_dispatcher.send_action_goal(position=self._target_position, time_from_start=0.02)
        self.assertGreater(_SPEED_LIMIT, self._max_frame_speed.get(), 'Speed limit of 0.25[m/s] was violated')

    def test_automatic_mode(self):
        rospy.loginfo('Test speed monitoring in AUTO mode')

        duration = self._compute_min_duration()

        self._max_frame_speed.reset()
        self._trajectory_dispatcher.send_action_goal(position=self._target_position, time_from_start=duration)
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
