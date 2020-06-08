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
import rospy

from actionlib_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

COMMAND_TOPIC_NAME = '/prbt/manipulator_joint_trajectory_controller/command'
FOLLOW_JOINT_TRAJ_ACTION_NAME = '/prbt/manipulator_joint_trajectory_controller/follow_joint_trajectory'
JOINT_NAMES = ['prbt_joint_1', 'prbt_joint_2', 'prbt_joint_3', 'prbt_joint_4', 'prbt_joint_5', 'prbt_joint_6']

DEFAULT_QUEUE_SIZE = 10
DEFAULT_TRAJECTORY_DURATION_S = 5
WAIT_FOR_SERVICE_TIMEOUT_S = 10

class SinglePointTrajectoryDispatcher:

    def __init__(self):
        self._pub = rospy.Publisher(COMMAND_TOPIC_NAME, JointTrajectory, queue_size=DEFAULT_QUEUE_SIZE)

        self._client = actionlib.SimpleActionClient(FOLLOW_JOINT_TRAJ_ACTION_NAME, FollowJointTrajectoryAction)

        timeout = rospy.Duration(WAIT_FOR_SERVICE_TIMEOUT_S)
        self._client.wait_for_server(timeout)

    def sendMessage(self, position=[0.0]*len(JOINT_NAMES), velocity=[], time_from_start = DEFAULT_TRAJECTORY_DURATION_S):
        assert len(position) == len(JOINT_NAMES)
        if velocity:
            assert len(velocity) == len(JOINT_NAMES)

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = position
        point.velocities = velocity
        point.time_from_start = rospy.Duration(time_from_start)

        traj.points.append(point)

        self._pub.publish(traj)

    def sendActionGoal(self, position=[0.0]*len(JOINT_NAMES), velocity=[], velocity_tolerance=0.0,
                       time_from_start = DEFAULT_TRAJECTORY_DURATION_S):
        assert len(position) == len(JOINT_NAMES)
        if velocity:
            assert len(velocity) == len(JOINT_NAMES)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = position
        point.velocities = velocity
        point.time_from_start = rospy.Duration(time_from_start)

        goal.trajectory.points = [point]

        for joint_name in JOINT_NAMES:
            tol = JointTolerance()
            tol.name = joint_name
            tol.velocity = velocity_tolerance
            goal.path_tolerance.append(tol)

        self._client.send_goal_and_wait(goal)


def askForPermission(test_case):
    s = raw_input('Perform ' + test_case + ' [(y)es, (n)o]?: ')
    return (s == "y")


def program():
    trajectory_dispatcher = SinglePointTrajectoryDispatcher()

    print('!!! BE CAREFUL. ROBOT MIGHT CRASH. !!!')

    rospy.sleep(3.0)

    print('Move to home position...')
    trajectory_dispatcher.sendActionGoal()

    start_position = [0.0, -0.5, 0.0, 0.0, 0.0, 0.0]
    target_position = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]

    if askForPermission('critical movement by topic'):
        print('First move to start position...')
        trajectory_dispatcher.sendActionGoal(position=start_position)
        trajectory_dispatcher.sendMessage(position=target_position, time_from_start=0.02)

    if askForPermission('critical movement by action'):
        print('First move to start position...')
        trajectory_dispatcher.sendActionGoal(position=start_position)
        trajectory_dispatcher.sendActionGoal(position=target_position, time_from_start=0.02)

    if askForPermission('critical movement by action with added tolerance'):
        print('First move to start position...')
        trajectory_dispatcher.sendActionGoal(position=start_position)
        trajectory_dispatcher.sendActionGoal(position=target_position, time_from_start=0.02, velocity_tolerance=1.0)

    if askForPermission('critical movement by action with added velocity'):
        print('First move to start position...')
        trajectory_dispatcher.sendActionGoal(position=start_position)
        trajectory_dispatcher.sendActionGoal(position=target_position, time_from_start=0.02, velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


if __name__ == "__main__":
    rospy.init_node('acceptance_test_stop_trajectory')
    program()
