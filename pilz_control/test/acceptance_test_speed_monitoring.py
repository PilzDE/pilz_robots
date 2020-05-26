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

import math
import rospy
import unittest

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger, TriggerRequest

# needs pilz_robot_programming
from pilz_robot_programming.robot import *
from pilz_robot_programming.commands import *

_MAX_FRAME_SPEED_TOPIC_NAME = '/max_frame_speed'
_SPEED_LIMIT = 0.25
_VEL_SCALE_DEFAULT = 0.5
_LONG_TRAJ_CMD_DURATION = 5.0

_DEFAULT_QUEUE_SIZE = 10
_JOINT_NAMES = ['prbt_joint_1', 'prbt_joint_2', 'prbt_joint_3', 'prbt_joint_4', 'prbt_joint_5', 'prbt_joint_6']

_UNHOLD_SERVICE_NAME = '/prbt/manipulator_joint_trajectory_controller/unhold'
_COMMAND_TOPIC_NAME = '/prbt/manipulator_joint_trajectory_controller/command'

_REQUIRED_API_VERSION = '1'


class AcceptancetestSpeedMonitoring(unittest.TestCase):
    """ Prerequisites: Launch robot and joint_states_speed_observer.
    """

    def setUp(self):
        """ Setup description
        """

        self.robot = Robot(_REQUIRED_API_VERSION)

        self._max_frame_speed_sub = rospy.Subscriber(_MAX_FRAME_SPEED_TOPIC_NAME, Float64, self._max_frame_speed_callback)

        self._command_pub = rospy.Publisher(_COMMAND_TOPIC_NAME, JointTrajectory, queue_size=_DEFAULT_QUEUE_SIZE)

        self._max_frame_speed = 0

        rospy.wait_for_service(_UNHOLD_SERVICE_NAME)
        self._unhold_service = rospy.ServiceProxy(_UNHOLD_SERVICE_NAME, Trigger)

    def tearDown(self):
        """ Teardown description
        """
        self.robot._release()

    def _trigger_unhold(self):
        req = TriggerRequest()
        self._unhold_service(req)

    def _reset_max_frame_speed(self):
        self._max_frame_speed = 0

    def _max_frame_speed_callback(self, msg):
        """ Detects the maximum speed of all monitored links
        """
        self._max_frame_speed = max(self._max_frame_speed, msg.data)

    def _send_trajectory_command_and_wait(self, position, time_from_start):
        """ Publish a trajectory command containing one point
            - position: Position of the joint2
        """
        traj = JointTrajectory()
        traj.joint_names = _JOINT_NAMES
        traj.header.stamp = rospy.Time.now()
        point = JointTrajectoryPoint()
        point.positions = [0.0, position, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = rospy.Duration(time_from_start)
        traj.points.append(point)

        self._command_pub.publish(traj)

        rospy.sleep(time_from_start)

    def test_speed_monitoring(self):
        """ Tests speed observing by sending trajectory commands.
            The target duration (representing the speed limit) is computed via a formula and one speed observation.
        """
        self._trigger_unhold()
        target_pos = -0.5
        self.robot.move(Ptp(goal=[0, target_pos, 0, 0, 0, 0], vel_scale=_VEL_SCALE_DEFAULT))

        self._reset_max_frame_speed()
        target_pos *= -1.0
        self._send_trajectory_command_and_wait(target_pos, _LONG_TRAJ_CMD_DURATION)

        target_duration = _LONG_TRAJ_CMD_DURATION * self._max_frame_speed / _SPEED_LIMIT

        print('Command movement below speed limit...')

        self._reset_max_frame_speed()
        target_pos *= -1.0
        time_from_start = 1.1 * target_duration
        self._send_trajectory_command_and_wait(target_pos, time_from_start)

        # in case movement failed
        self._trigger_unhold()
        self._send_trajectory_command_and_wait(target_pos, _LONG_TRAJ_CMD_DURATION)

        print('Movement had maximum frame speed: ' + str(self._max_frame_speed))

        print('Command movement above speed limit...')

        self._reset_max_frame_speed()
        target_pos *= -1.0
        time_from_start = 0.9 * target_duration
        self._send_trajectory_command_and_wait(target_pos, time_from_start)

        print('Movement had maximum frame speed: ' + str(self._max_frame_speed))


if __name__ == "__main__":
    rospy.init_node('acceptancetest_speed_monitoring')
    unittest.main()
