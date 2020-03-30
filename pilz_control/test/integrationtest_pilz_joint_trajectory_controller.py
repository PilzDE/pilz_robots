#!/usr/bin/env python

# Copyright (c) 2018 Pilz GmbH & Co. KG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
import rospy
import threading
from rospy.exceptions import ROSException
from std_srvs.srv import Trigger, TriggerRequest, SetBool, SetBoolRequest
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint

import actionlib
from actionlib_msgs.msg import *

PACKAGE_NAME = 'pilz_control'
CONTROLLER_NS_PARAM_NAME = 'controller_ns_string'
JOINT_NAMES = ['shoulder_to_right_arm', 'shoulder_to_left_arm']
HOLD_SERVICE_NAME = '/test_joint_trajectory_controller/hold'
UNHOLD_SERVICE_NAME = '/test_joint_trajectory_controller/unhold'
IS_EXECUTING_SERVICE_NAME = '/test_joint_trajectory_controller/is_executing'
CARTESIAN_SPEED_SERVICE_NAME = '/test_joint_trajectory_controller/monitor_cartesian_speed'
ACTION_NAME = '/test_joint_trajectory_controller/follow_joint_trajectory'
STATE_TOPIC_NAME = '/test_joint_trajectory_controller/state'

WAIT_FOR_SERVICE_TIMEOUT_S = 10
SLEEP_RATE_HZ = 10
STOP_DURATION_UPPER_BOUND_S = 0.31
MAX_DECELERATION = 7.85

# Default goal position of the test trajectory
DEFAULT_GOAL_POSITION = [0.1, 0]

controller_ns = rospy.get_param(CONTROLLER_NS_PARAM_NAME)


class ObservationException(Exception):
    """Exception class used by MovementObserver"""
    pass


class MovementObserver:
    """Class that subscribes to the robot mock state to observe the movement.

    Enables to run blocking observations until a condition has been fulfilled or a timeout passed
    :note Current implementation assumes that the robot mock only has one joint.

    """
    def __init__(self):
        self._actual_position = None
        self._actual_velocity = None
        self._last_time_stamp = 0.0
        self._controller_state_delta_t = None
        self._controller_state_lock = threading.Lock()

        state_topic_name = controller_ns + STATE_TOPIC_NAME
        self._subscriber = rospy.Subscriber(state_topic_name, JointTrajectoryControllerState,
                                            self.controller_state_callback)

        self._stop_observer_thread_lock = threading.Lock()
        self._stop_observer_thread = None

        self._stop_trajectory_ok_lock = threading.Lock()
        self._stop_trajectory_ok = False

    def controller_state_callback(self, data):
        with self._controller_state_lock:
            self._actual_position = data.actual.positions
            self._actual_velocity = data.actual.velocities
            self._controller_state_delta_t = data.header.stamp.to_sec() - self._last_time_stamp
            self._last_time_stamp = data.header.stamp.to_sec()

    def _is_position_threshold_reached(self, position_threshold):
        with self._controller_state_lock:
            if self._actual_position is None:
                return False
            current_pos = list(self._actual_position)

        for i in range(len(position_threshold)):
            if current_pos[i] < position_threshold[i]:
                return False
        return True

    @staticmethod
    def _is_deceleration_limit_violated(actual_velocity, old_velocity, delta_t, max_deceleration):
        for i in range(len(actual_velocity)):
            if ((old_velocity[i] - actual_velocity[i]) * delta_t) > max_deceleration:
                return True
        return False

    @staticmethod
    def _is_stop_motion_finished(actual_velocity):
        for i in range(len(actual_velocity)):
            if abs(actual_velocity[i]) != 0.0:
                return False
        return True

    def observe_until_position_greater(self, position_threshold, timeout):
        """ Blocking observation until the observed position is above the position_threshold. """
        r = rospy.Rate(SLEEP_RATE_HZ)
        start_loop = rospy.Time.now()
        while not self._is_position_threshold_reached(position_threshold):
            r.sleep()
            if timeout < (rospy.Time.now() - start_loop).to_sec():
                raise ObservationException("Position not above defined threshold within timeout")

    def _observe_stop_trajectory(self, max_deceleration=MAX_DECELERATION, timeout=STOP_DURATION_UPPER_BOUND_S):
        """ Observes that the robot stops within the specified timeout while the deceleration stays below
        the maximum deceleration.

        :param max_deceleration: Max allowed deceleration. The deceleration has to be positive.
        :param timeout: Time in which robot has to stop.
        """
        r = rospy.Rate(SLEEP_RATE_HZ)
        start_stop = rospy.Time.now()

        # Main observation loop, not infinite due to timeout
        while True:
            stop_duration = (rospy.Time.now() - start_stop).to_sec()
            if timeout < stop_duration:
                with self._stop_trajectory_ok_lock:
                    self._stop_trajectory_ok = False
                rospy.logerr('Stop lasted too long: ' + str(stop_duration) + ' seconds.')
                return

            # Obtain local of actual velocity
            with self._controller_state_lock:
                # Noting to be done if no actual_velocity was observed
                if self._actual_velocity is None:
                    continue
                actual_velocity = list(self._actual_velocity)

            if self._is_stop_motion_finished(actual_velocity):
                with self._stop_trajectory_ok_lock:
                    self._stop_trajectory_ok = True
                return

            old_velocity = actual_velocity

            r.sleep()

            # Check for abrupt stop
            with self._controller_state_lock:
                limit_violated = self._is_deceleration_limit_violated(self._actual_velocity,
                                                                      old_velocity,
                                                                      self._controller_state_delta_t,
                                                                      max_deceleration)
            if limit_violated:
                with self._stop_trajectory_ok_lock:
                    self._stop_trajectory_ok = False
                rospy.logerr("Abrupt stop detected")
                return

    def start_stop_observation(self):
        with self._stop_observer_thread_lock:
            if self._stop_observer_thread is not None:
                rospy.logerr("Somebody did already trigger stop observation")
                return
            self._stop_observer_thread = threading.Thread(target=self._observe_stop_trajectory)
            self._stop_observer_thread.start()

    def wait_for_end_of_stop_observation(self):
        with self._stop_observer_thread_lock:
            if self._stop_observer_thread is None:
                return False
            self._stop_observer_thread.join()
            self._stop_observer_thread = None

        with self._stop_trajectory_ok_lock:
            return self._stop_trajectory_ok


class TrajectoryDispatcher:
    """A wrapper around the SimpleActionClient for dispatching trajectories."""
    def __init__(self):
        action_name = controller_ns + ACTION_NAME
        self._client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)

        timeout = rospy.Duration(WAIT_FOR_SERVICE_TIMEOUT_S)
        self._client.wait_for_server(timeout)

    def dispatch_trajectory(self, goal_position, time_from_start):
        """Sends a simple JointTrajectory to the action client.

        :param goal_position: The only position in the send trajectory
        :param time_from_start: The time of the only position to be achieved (Starting at 0)
        """
        point = JointTrajectoryPoint()
        point.positions = goal_position
        point.time_from_start = rospy.Duration(time_from_start)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.points = [point]
        goal.trajectory.joint_names = JOINT_NAMES

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


class SetMonitoredCartesianSpeed:
    def __init__(self):
        service_name = controller_ns + CARTESIAN_SPEED_SERVICE_NAME
        rospy.wait_for_service(service_name, WAIT_FOR_SERVICE_TIMEOUT_S)
        self._set_catesian_speed_srv = rospy.ServiceProxy(service_name, SetBool)

    def _call(self, monitoring_on_off_flag):
        req = SetBoolRequest()
        req.data = monitoring_on_off_flag
        resp = self._set_catesian_speed_srv(req)
        return resp.success

    def turn_off_speed_monitoring(self):
        return self._call(False)

    def turn_on_speed_monitoring(self):
        return self._call(True)


class IsExecutingServiceWrapper:
    """Wrapper for the service querying if the controller is executing."""
    def __init__(self):
        is_executing_service_name = controller_ns + IS_EXECUTING_SERVICE_NAME
        rospy.wait_for_service(is_executing_service_name, WAIT_FOR_SERVICE_TIMEOUT_S)
        self._is_executing_srv = rospy.ServiceProxy(is_executing_service_name, Trigger)

    def call(self):
        req = TriggerRequest()
        resp = self._is_executing_srv(req)

        return resp.success


class StopServiceWrapper:
    """Abstraction around the service call to switch the controller between DEFAULT and HOLDING mode."""
    def __init__(self):
        hold_service_name = controller_ns + HOLD_SERVICE_NAME
        rospy.wait_for_service(hold_service_name, WAIT_FOR_SERVICE_TIMEOUT_S)
        self._hold_srv = rospy.ServiceProxy(hold_service_name, Trigger)

        unhold_service_name = controller_ns + UNHOLD_SERVICE_NAME
        rospy.wait_for_service(unhold_service_name, WAIT_FOR_SERVICE_TIMEOUT_S)
        self._unhold_srv = rospy.ServiceProxy(unhold_service_name, Trigger)

    def request_default_mode(self):
        """Switch into DEFAULT mode by sending the respective request.

        :return: True if service request was handled successful, False otherwise.
        """
        req = TriggerRequest()
        resp = self._unhold_srv(req)

        return resp.success

    def request_holding_mode(self):
        """Switch into HOLDING mode by sending the respective request.

        :return: True if service request was handled successful, False otherwise
        """
        req = TriggerRequest()
        resp = self._hold_srv(req)

        return resp.success


class TestPilzJointTrajectoryController(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestPilzJointTrajectoryController, self).__init__(*args, **kwargs)

        self._trajectory_dispatcher = TrajectoryDispatcher()
        self._monitored_cartesian_speed_srv = SetMonitoredCartesianSpeed()
        self._hold_srv = StopServiceWrapper()

    def setUp(self):
        self.assertTrue(self._turn_off_speed_monitoring(), 'Could not turn off speed monitoring')

    def _move_to(self, position, duration, expected_error_code=FollowJointTrajectoryResult.SUCCESSFUL):
        self._start_motion(position=position, duration=duration)
        return self._wait_for_motion_result(expected_error_code)

    def _start_motion(self, position, duration):
        self._trajectory_dispatcher.dispatch_trajectory(goal_position=position, time_from_start=duration)

    def _wait_for_motion_result(self, expected_error_code=FollowJointTrajectoryResult.SUCCESSFUL):
        result = self._trajectory_dispatcher.wait_for_result()
        return result.error_code == expected_error_code

    def _turn_off_speed_monitoring(self):
        return self._monitored_cartesian_speed_srv.turn_off_speed_monitoring()

    def _turn_on_speed_monitoring(self):
        return self._monitored_cartesian_speed_srv.turn_on_speed_monitoring()

    def test_hold_at_startup(self):
        """Tests that 'hold' mode is active at startup of controller."""

        self.assertTrue(self._move_to(position=DEFAULT_GOAL_POSITION, duration=1,
                                      expected_error_code=FollowJointTrajectoryResult.INVALID_GOAL),
                        "Motion did not fail although controller should be in 'hold' mode at startup")

        if not self._hold_srv.request_default_mode():
            rospy.sleep(STOP_DURATION_UPPER_BOUND_S)
        self.assertTrue(self._hold_srv.request_default_mode(), 'Switch to default mode failed')

        self.assertTrue(self._move_to(position=DEFAULT_GOAL_POSITION, duration=0.5),
                        "Motion failed although controller should be in 'unhold' mode")

    def test_hold_during_motion(self):
        """Activate hold mode while robot is moving and evaluate executed stop."""
        is_executing_srv = IsExecutingServiceWrapper()

        new_pos = list(DEFAULT_GOAL_POSITION)
        new_pos[0] += 0.1

        self._start_motion(position=new_pos, duration=5)

        # Wait for movement to commence
        motion_observer = MovementObserver()
        motion_observer.observe_until_position_greater([0.11, 0.0], 3)
        self.assertTrue(is_executing_srv.call(), "Controller is not executing")
        # Make sure robot stops (not abruptly)
        motion_observer.start_stop_observation()
        rospy.loginfo("Robot is moving -> Switch into HOLDING mode")
        self.assertTrue(self._hold_srv.request_holding_mode(), "Switch to Mode HOLDING failed.")
        self.assertTrue(motion_observer.wait_for_end_of_stop_observation(), "Stop trajectory incorrect")
        rospy.loginfo("Stop of robot observed")

        self.assertTrue(self._wait_for_motion_result(), "Motion failed")

        self.assertEqual(GoalStatus.PREEMPTED, self._trajectory_dispatcher.get_last_state(), "Goal was not preempted")

    def test_speed_monitoring(self):
        """Tests if controller detects Cartesian speed limit violation and switches to hold mode."""
        self.assertTrue(self._turn_on_speed_monitoring(), "Could not turn on speed monitoring")

        # Test speed monitoring for all joints
        for i in range(len(JOINT_NAMES)):
            far_away_position = list(DEFAULT_GOAL_POSITION)
            far_away_position[i] += 0.4

            self.assertTrue(self._hold_srv.request_default_mode(), "Switch to default mode failed")

            self.assertTrue(self._move_to(position=DEFAULT_GOAL_POSITION, duration=0.5),
                            "Motion to default position failed")

            move_result = self._move_to(position=far_away_position, duration=0.1)
            # Unfortunately, the goal returns SUCCESSFUL as error_code, in case the motion
            # is cancelled. Therefore, the following check is commented out.
            # self.assertFalse(move_result, "Cartesian speed monitor did not detect speed limit violation")

            self.assertEqual(GoalStatus.PREEMPTED, self._trajectory_dispatcher.get_last_state(), 'Goal was not preempted.')
            motion_observer = MovementObserver()
            # Make sure robot stops (not abruptly)
            motion_observer.start_stop_observation()
            self.assertTrue(motion_observer.wait_for_end_of_stop_observation(), "Stop trajectory incorrect")

            self.assertTrue(self._move_to(position=far_away_position, duration=0.5,
                                          expected_error_code=FollowJointTrajectoryResult.INVALID_GOAL),
                            "Controller did not block motion execution although controller should be in 'hold' mode")

            self.assertTrue(self._hold_srv.request_default_mode(), "Switch to 'unhold' mode failed")

            self.assertTrue(self._move_to(position=far_away_position, duration=0.5),
                            "Controller did not allow motion execution, although controller should be in 'unhold' mode")


if __name__ == '__main__':
    import rostest
    rospy.init_node('integrationtest_pilz_joint_trajectory_controller')
    rostest.rosrun(PACKAGE_NAME, 'integrationtest_pilz_joint_trajectory_controller', TestPilzJointTrajectoryController)
