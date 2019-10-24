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
from std_srvs.srv import Trigger, TriggerRequest
from prbt_hardware_support.msg import FrameSpeeds

# needs pilz_robot_programming
from pilz_robot_programming.robot import *
from pilz_robot_programming.commands import *

_FRAME_SPEEDS_TOPIC_NAME = '/prbt/joint_states_frame_speeds'
_OUTMOST_LINK_NAME = 'prbt_flange'
_JOINT_VELOCITY_LIMIT = 1.57
_SPEED_LIMIT = 0.25

_UNHOLD_SERVICE_NAME = '/prbt/manipulator_joint_trajectory_controller/unhold'

_REQUIRED_API_VERSION = '1'


class AcceptancetestSpeedObserver(unittest.TestCase):
    """ Prerequisites: Launch robot and joint_states_speed_observer, make sure no further speed scaling is applied.

        Tests speed observing by performing multiple scaled PTP motions.
        The target scale (representing the speed limit) is computed via a formula and one speed observation.

        Currently, the results are just printed.
    """

    def setUp(self):
        """ Setup description
        """

        self.robot = Robot(_REQUIRED_API_VERSION)

        msg = rospy.wait_for_message(_FRAME_SPEEDS_TOPIC_NAME, FrameSpeeds)
        self.assertTrue(_OUTMOST_LINK_NAME in msg.name)
        self._outmost_link_index = msg.name.index(_OUTMOST_LINK_NAME)

        self._frame_speed_sub = rospy.Subscriber(_FRAME_SPEEDS_TOPIC_NAME, FrameSpeeds, self._frame_speeds_callback)

        self._max_frame_speed = 0

        rospy.wait_for_service(_UNHOLD_SERVICE_NAME)
        self._unhold_service = rospy.ServiceProxy(_UNHOLD_SERVICE_NAME, Trigger)

    def _frame_speeds_callback(self, msg):
        """ Detects the maximum speed of the outmost link
        """
        current_speed = msg.speed[self._outmost_link_index]
        self._max_frame_speed = max(self._max_frame_speed, current_speed)

    def test_speed_observer(self):
        """ Assuming no further speed scaling is applied
        """
        req = TriggerRequest()
        self._unhold_service(req)
        self.robot.move(Ptp(goal=[0, -0.5, 0, 0, 0, 0], vel_scale=0.1))

        self._max_frame_speed = 0

        self.robot.move(Ptp(goal=[0, 0.5, 0, 0, 0, 0], vel_scale=0.1))

        angular_vel = 0.1 * _JOINT_VELOCITY_LIMIT
        radius = self._max_frame_speed / angular_vel

        target_angular_vel = _SPEED_LIMIT / radius
        target_vel_scale = target_angular_vel / _JOINT_VELOCITY_LIMIT

        print('Start PTP movements around the speed limit.')

        target_pos = -0.5

        for scaling_factor in [0.9, 0.98, 0.99, 1.0, 1.01, 1.02, 1.1]:

            self._max_frame_speed = 0
            vel_scale = scaling_factor * target_vel_scale
            movement_failed = False
            try:
                self.robot.move(Ptp(goal=[0, target_pos, 0, 0, 0, 0], vel_scale=vel_scale))
                target_pos *= -1.0
            except RobotMoveFailed:
                movement_failed = True
                self._unhold_service(req)

            print('Movement ' + ('failed' if movement_failed else 'succeeded') + ' with maximum frame speed: ' + str(self._max_frame_speed))


if __name__ == "__main__":
    rospy.init_node('acceptancetest_speed_observer')
    unittest.main()
