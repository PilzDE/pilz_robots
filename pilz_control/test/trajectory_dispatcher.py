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

import actionlib
import math
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryDispatcher:
    """A wrapper around the SimpleActionClient for dispatching trajectories."""

    WAIT_FOR_SERVICE_TIMEOUT_S = 10
    DEFAULT_TRAJECTORY_DURATION_S = 10

    FOLLOW_JOINT_TRAJECTORY_SUFFIX = "/follow_joint_trajectory"
    JOINT_NAMES_SUFFIX = "/joint_names"
    FLOAT_EPSILON = 1e-9

    def __init__(self, controller_ns, controller_name):
        action_name = controller_ns + "/" + controller_name + self.FOLLOW_JOINT_TRAJECTORY_SUFFIX
        self._client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)

        parameter_name = controller_ns + self.JOINT_NAMES_SUFFIX
        self._joint_names = rospy.get_param(parameter_name)

        timeout = rospy.Duration(self.WAIT_FOR_SERVICE_TIMEOUT_S)
        self._client.wait_for_server(timeout)

    def dispatch_single_point_trajectory(self, goal_position, goal_velocity=[],
                                         time_from_start=DEFAULT_TRAJECTORY_DURATION_S):
        """Sends a simple JointTrajectory to the action client.

        :param goal_position: The only position in the send trajectory
        :param goal_velocity: The only velocity in the send trajectory.
                              If not set the controller will do a linear interpolation.
        :param time_from_start: The time of the only position to be achieved (Starting at 0)
        """
        assert len(goal_position) == len(self._joint_names)
        if goal_velocity:
            assert len(goal_velocity) == len(self._joint_names)

        point = JointTrajectoryPoint()
        point.positions = goal_position
        point.velocities = goal_velocity
        point.time_from_start = rospy.Duration(time_from_start)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.points = [point]
        goal.trajectory.joint_names = self._joint_names

        self._client.send_goal(goal)

    def dispatch_single_point_continuous_trajectory(self, goal_position,
                                                    time_from_start=DEFAULT_TRAJECTORY_DURATION_S):
        """Sends a simple JointTrajectory to the action client.
        Set the velocities such that the resulting trajectory has a continuous velocity.

        :param goal_position: The only position in the send trajectory
        :param time_from_start: The time of the only position to be achieved (Starting at 0)
        """
        self.dispatch_single_point_trajectory(goal_position, [0.] * len(self._joint_names), time_from_start)

    def dispatch_dual_point_trajectory(self, joint_index, start_position, mid_position, end_position, velocity):
        """ Send a trajectory with two points. The first part serves for accelerating to the specified velocity.
            The second part is the actual movement that is linear in joint space.
            Note that this is a single-joint-movement.

        :param joint_index: Only the joint given by joint_index will be moved.
                            This is an index into the joint names given on the parameter server.
        :param start_position: The current position of the joint. This will not be appended to the trajectory.
        :param mid_position: Joint position where the target velocity should be reached.
        :param end_position: Joint position at the end of the trajectory.
        :param velocity: Target joint velocity.
        """
        assert(velocity > self.FLOAT_EPSILON)

        acc_part_position_diff = mid_position - start_position
        lin_part_position_diff = end_position - mid_position
        assert(math.copysign(1.0, velocity) == math.copysign(1.0, acc_part_position_diff))
        assert(math.copysign(1.0, velocity) == math.copysign(1.0, lin_part_position_diff))

        lin_part_duration = lin_part_position_diff / velocity
        acc_part_duration = 2 * acc_part_position_diff / velocity

        mid_positions = [0.0]*len(self._joint_names)
        mid_positions[joint_index] = mid_position
        end_positions = [0.0]*len(self._joint_names)
        end_positions[joint_index] = end_position
        velocities = [0.0]*len(self._joint_names)
        velocities[joint_index] = velocity

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self._joint_names

        mid_point = JointTrajectoryPoint()
        mid_point.positions = mid_positions
        mid_point.velocities = velocities
        mid_point.time_from_start = rospy.Duration(acc_part_duration)

        end_point = JointTrajectoryPoint()
        end_point.positions = end_positions
        end_point.velocities = velocities
        end_point.time_from_start = rospy.Duration(acc_part_duration + lin_part_duration)

        goal.trajectory.points = [mid_point, end_point]

        self._client.send_goal(goal)

    def wait_for_result(self):
        """Wait until the result of the trajectory execution is received."""
        self._client.wait_for_result()
        return self._client.get_result()

    def get_last_state(self):
        """Get the state of the last send trajectory

        :return: see http://docs.ros.org/melodic/api/actionlib_msgs/html/msg/GoalStatus.html
        """
        return self._client.get_state()
