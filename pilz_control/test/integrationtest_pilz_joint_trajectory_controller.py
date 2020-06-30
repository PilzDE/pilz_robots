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

from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Trigger, TriggerRequest, SetBool, SetBoolRequest
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint

from controller_state_observer import ControllerStateObserver
from holding_mode_service_wrapper import HoldingModeServiceWrapper
from trajectory_dispatcher import TrajectoryDispatcher

PACKAGE_NAME = 'pilz_control'
CONTROLLER_NS_PARAM_NAME = 'controller_ns_string'
JOINT_NAMES = ['shoulder_to_right_arm', 'shoulder_to_left_arm']
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
DEFAULT_GOAL_DURATION_S = 5
DEFAULT_GOAL_POSITION_DELTA = .1

CONTROLLER_NS = rospy.get_param(CONTROLLER_NS_PARAM_NAME)
CONTROLLER_NAME = 'test_joint_trajectory_controller'


class ObservationException(Exception):
    """Exception class used by MovementObserver"""
    pass


class MovementObserver:
    """Class that subscribes to the robot mock state to observe the movement.

    Enables to run blocking observations until a condition has been fulfilled or a timeout passed
    """
    def __init__(self):
        self._state_observer = ControllerStateObserver(CONTROLLER_NS, CONTROLLER_NAME)

        self._stop_observer_thread_lock = threading.Lock()
        self._stop_observer_thread = None

        self._stop_trajectory_ok_lock = threading.Lock()
        self._stop_trajectory_ok = False

    def _is_position_threshold_reached(self, position_threshold):
        current_pos = self._state_observer.get_actual_position()

        for i in range(len(position_threshold)):
            if current_pos[i] < position_threshold[i]:
                return False
        return True

    @staticmethod
    def _is_deceleration_limit_violated(actual_deceleration, max_deceleration):
        for val in actual_deceleration:
            if val > max_deceleration:
                return True
        return False

    @staticmethod
    def _is_stop_motion_finished(actual_velocity):
        for val in actual_velocity:
            if abs(val) != 0.0:
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

            actual_velocity = self._state_observer.get_actual_velocity()
            if self._is_stop_motion_finished(actual_velocity):
                with self._stop_trajectory_ok_lock:
                    self._stop_trajectory_ok = True
                return

            r.sleep()

            # Check for abrupt stop
            actual_deceleration = -self._state_observer.get_actual_acceleration()
            limit_violated = self._is_deceleration_limit_violated(actual_deceleration, max_deceleration)
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

    def get_actual_position(self):
        return self._state_observer.get_actual_position()


class SetMonitoredCartesianSpeed:
    def __init__(self):
        service_name = CONTROLLER_NS + CARTESIAN_SPEED_SERVICE_NAME
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
        is_executing_service_name = CONTROLLER_NS + IS_EXECUTING_SERVICE_NAME
        rospy.wait_for_service(is_executing_service_name, WAIT_FOR_SERVICE_TIMEOUT_S)
        self._is_executing_srv = rospy.ServiceProxy(is_executing_service_name, Trigger)

    def call(self):
        req = TriggerRequest()
        resp = self._is_executing_srv(req)

        return resp.success


class TestPilzJointTrajectoryController(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestPilzJointTrajectoryController, self).__init__(*args, **kwargs)

        self._trajectory_dispatcher = TrajectoryDispatcher(CONTROLLER_NS, CONTROLLER_NAME)
        self._monitored_cartesian_speed_srv = SetMonitoredCartesianSpeed()
        self._hold_srv = HoldingModeServiceWrapper(CONTROLLER_NS, CONTROLLER_NAME)

    def setUp(self):
        self.assertTrue(self._turn_off_speed_monitoring(), 'Could not turn off speed monitoring')

    def _move_to(self, position, duration, expected_error_code=FollowJointTrajectoryResult.SUCCESSFUL):
        self._start_motion(position=position, duration=duration)
        return self._wait_for_motion_result(expected_error_code)

    def _start_motion(self, position, duration):
        self._trajectory_dispatcher.dispatch_single_point_trajectory(goal_position=position, time_from_start=duration)

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

        self.assertTrue(self._move_to(position=DEFAULT_GOAL_POSITION, duration=DEFAULT_GOAL_DURATION_S),
                        "Motion failed although controller should be in 'unhold' mode")

    def test_hold_during_motion(self):
        """Activate hold mode while robot is moving and evaluate executed stop."""
        is_executing_srv = IsExecutingServiceWrapper()
        motion_observer = MovementObserver()
        start_pose = motion_observer.get_actual_position()
        rospy.loginfo("start_pose: " + str(start_pose))

        new_pos = list(DEFAULT_GOAL_POSITION)
        new_pos[0] += DEFAULT_GOAL_POSITION_DELTA

        self._start_motion(position=new_pos, duration=DEFAULT_GOAL_DURATION_S)

        # Wait for movement to commence
        motion_observer.observe_until_position_greater(start_pose, DEFAULT_GOAL_DURATION_S / 2.)
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
        motion_observer = MovementObserver()

        # Test speed monitoring for all joints
        for i in range(len(JOINT_NAMES)):
            far_away_position = list(DEFAULT_GOAL_POSITION)
            far_away_position[i] += DEFAULT_GOAL_POSITION_DELTA

            self.assertTrue(self._hold_srv.request_default_mode(), "Switch to default mode failed")

            self.assertTrue(self._move_to(position=DEFAULT_GOAL_POSITION, duration=DEFAULT_GOAL_DURATION_S),
                            "Motion to default position failed")

            move_result = self._move_to(position=far_away_position, duration=DEFAULT_GOAL_DURATION_S / 3.)
            # Unfortunately, the goal returns SUCCESSFUL as error_code, in case the motion
            # is cancelled. Therefore, the following check is commented out.
            # self.assertFalse(move_result, "Cartesian speed monitor did not detect speed limit violation")

            self.assertEqual(GoalStatus.ABORTED, self._trajectory_dispatcher.get_last_state(),
                             'Goal was not preempted.')
            motion_observer = MovementObserver()
            # Make sure robot stops (not abruptly)
            motion_observer.start_stop_observation()
            self.assertTrue(motion_observer.wait_for_end_of_stop_observation(), "Stop trajectory incorrect")

            self.assertTrue(self._move_to(position=far_away_position, duration=DEFAULT_GOAL_DURATION_S,
                                          expected_error_code=FollowJointTrajectoryResult.INVALID_GOAL),
                            "Controller did not block motion execution although controller should be in 'hold' mode")

            if not self._hold_srv.request_default_mode():
                # Stop motion has finished but controller could still be in stopping mode
                rospy.sleep(STOP_DURATION_UPPER_BOUND_S)
            self.assertTrue(self._hold_srv.request_default_mode(), "Switch to 'unhold' mode failed")

            self.assertTrue(self._move_to(position=far_away_position, duration=DEFAULT_GOAL_DURATION_S),
                            "Controller did not allow motion execution, although controller should be in 'unhold' mode")


if __name__ == '__main__':
    import rostest
    rospy.init_node('integrationtest_pilz_joint_trajectory_controller')
    rostest.rosrun(PACKAGE_NAME, 'integrationtest_pilz_joint_trajectory_controller', TestPilzJointTrajectoryController)
