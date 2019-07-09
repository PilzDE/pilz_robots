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
from std_srvs.srv import Trigger, TriggerRequest
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint

import actionlib
from actionlib_msgs.msg import *

PACKAGE_NAME = 'pilz_control'
CONTROLLER_NS_PARAM_NAME = 'controller_ns_string'
JOINT_NAMES = ['joint1']
HOLD_SERVICE_NAME = '/test_joint_trajectory_controller/hold'
UNHOLD_SERVICE_NAME = '/test_joint_trajectory_controller/unhold'
IS_EXECUTING_SERVICE_NAME = '/test_joint_trajectory_controller/is_executing'
ACTION_NAME = '/test_joint_trajectory_controller/follow_joint_trajectory'
STATE_TOPIC_NAME = '/test_joint_trajectory_controller/state'

WAIT_FOR_SERVICE_TIMEOUT_S = 10
SLEEP_RATE_HZ = 10
STOP_DURATION_UPPER_BOUND_S = 0.31
MAX_DECELERATION = 0.2

# Default goal position of the test trajectory
DEFAULT_GOAL_POSITION = 0.1

controller_ns = rospy.get_param(CONTROLLER_NS_PARAM_NAME)

## Exception class used by MovementObserver
class ObservationException(Exception):
  pass

## Class that subscribes to the robot mock state to observe the movement.
#  Enables to run blocking observations until a condition has been fulfilled or a timeout passed
#
# \note Current implementation assumes that the robot mock only has one joint.
class MovementObserver:

    ## Upon construction subscribes to the state topic.
    def __init__(self):
        self._actual_position = None
        self._actual_position_lock = threading.Lock()
        self._actual_velocity = None
        self._actual_velocity_lock = threading.Lock()

        state_topic_name = controller_ns + STATE_TOPIC_NAME
        self._subscriber = rospy.Subscriber(state_topic_name, JointTrajectoryControllerState, self.controllerStateCallback)

    ## Callback use by the state subscription.
    def controllerStateCallback(self, data):
        with self._actual_position_lock:
            self._actual_position = data.actual.positions[0]
        with self._actual_velocity_lock:
            self._actual_velocity = data.actual.velocities[0]

    def _checkPositionThreshold(self, position_threshold):
        with self._actual_position_lock:
            return (self._actual_position is None) or (self._actual_position < position_threshold)

    ## Blocking observation until the observed position is above the \p position_threshold
    #  @throw \class ObservationException position
    def observeUntilPositionGreater(self, position_threshold, timeout):
        r = rospy.Rate(SLEEP_RATE_HZ)
        start_loop = rospy.Time.now()
        while self._checkPositionThreshold(position_threshold):
            r.sleep()
            if(timeout < (rospy.Time.now() - start_loop).to_sec()):
              raise ObservationException("Position not above defined threshold within timeout")

    ## Blocking observation that the robot stops within \p timeout seconds, ensures that the deceleration stays below
    #  \p max_deceleration
    #  @throw ObservationException
    #  @note \p max_deceleration is defined positive
    def observeStop(self, max_deceleration = MAX_DECELERATION, timeout = STOP_DURATION_UPPER_BOUND_S):
        r = rospy.Rate(SLEEP_RATE_HZ)
        start_stop = rospy.Time.now()

        # Main observation loop, not infinite due to timeout
        while True:

            # Obtain local of actual velocity
            actual_velocity = None
            with self._actual_position_lock:
                actual_velocity = self._actual_velocity

            # Timeout check
            if(timeout < (rospy.Time.now() - start_stop).to_sec()):
                raise ObservationException('Stop lasted too long: ' + str(stop_duration.to_sec()) + ' seconds.')

            # Noting to be done if no actual_velocity was observed
            if actual_velocity is None:
                continue

            # Check if stopped
            if abs(actual_velocity) == 0.0:
                return True

            old_velocity = actual_velocity

            r.sleep()

            # Check for abrupt stop
            with self._actual_position_lock:
                if(max_deceleration < abs(self._actual_velocity - old_velocity) * SLEEP_RATE_HZ):
                    raise ObservationException("Abrupt stop detected!")


## Essentially a wrapper around a SimpleActionClient for dispatching simple trajectories.
class TrajectoryDispatcher:

  def __init__(self):
      action_name = controller_ns + ACTION_NAME
      self._client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)

      timeout = rospy.Duration(WAIT_FOR_SERVICE_TIMEOUT_S)
      self._client.wait_for_server(timeout)

  ## Sends a simple JointTrajectory to the action client
  #  @param goal_position The only position in the send trajectory
  #  @param time_from_start The time of the only position to be achieved (Starting at 0)
  def dispatchTrajectory(self, goal_position, time_from_start):
      point = JointTrajectoryPoint()
      point.positions = [goal_position]
      point.time_from_start = rospy.Duration(time_from_start)

      goal = FollowJointTrajectoryGoal()
      goal.trajectory.header.stamp = rospy.Time.now()
      goal.trajectory.points = [point]
      goal.trajectory.joint_names = JOINT_NAMES

      self._client.send_goal(goal)

  ## Wait until the result of the trajectory execution is received
  def waitForResult(self):
      self._client.wait_for_result()
      return self._client.get_result()

  ## Get the state of the last send trajectory
  #  @return GoalStatus
  #  see <a href="http://docs.ros.org/melodic/api/actionlib_msgs/html/msg/GoalStatus.html">
  #       http://docs.ros.org/melodic/api/actionlib_msgs/html/msg/GoalStatus.html</a>
  #  @note Check result with e.g.
  #        \code self.assertEqual(GoalStatus.PREEMPTED, dispatcher.getLastState(), "Unexpected result code") \endcode
  def getLastState(self):
      return self._client.get_state()


## Wrapper for the service querying if the controller is executing
class IsExecutingServiceWrapper:
  def __init__(self):

      is_executing_service_name = controller_ns + IS_EXECUTING_SERVICE_NAME
      rospy.wait_for_service(is_executing_service_name, WAIT_FOR_SERVICE_TIMEOUT_S)
      self._is_executing_srv = rospy.ServiceProxy(is_executing_service_name, Trigger)

  def call(self):
      req = TriggerRequest()
      resp = self._is_executing_srv(req)

      return resp.success


## Abstraction around the service call for switch the controller between DEFAULT and HOLDING mode
class StopServiceWrapper:
  def __init__(self):

      hold_service_name = controller_ns + HOLD_SERVICE_NAME
      rospy.wait_for_service(hold_service_name, WAIT_FOR_SERVICE_TIMEOUT_S)
      self._hold_srv = rospy.ServiceProxy(hold_service_name, Trigger)

      unhold_service_name = controller_ns + UNHOLD_SERVICE_NAME
      rospy.wait_for_service(unhold_service_name, WAIT_FOR_SERVICE_TIMEOUT_S)
      self._unhold_srv = rospy.ServiceProxy(unhold_service_name, Trigger)

  ## Switch into DEFAULT mode by sending the respective request
  #  @return True if service request was handled successful, False otherwise
  def requestDefaultMode(self):
      req = TriggerRequest()
      resp = self._unhold_srv(req)

      return resp.success

  ## Switch into HOLDING mode by sending the respective request
  #  @return True if service request was handled successful, False otherwise
  def requestHoldingMode(self):
      req = TriggerRequest()
      resp = self._hold_srv(req)

      return resp.success

## Test the influence of the HOLD mode on joint trajectory goals. For this a small robot-mock is started and the
#  controller is spawned via the test launch file.
class IntegrationtestPilzJointTrajectoryController(unittest.TestCase):

    ## Test Sequence:
    #        1. Set up service proxy and action client.
    #        2. Send goal to controller action server.
    #        3. Request switch to DEFAULT mode via service.
    #        4. Send goal to controller action server.
    #        5. Send new goal to controller action server and switch to HOLDING during execution.
    #
    #    Expected Results:
    #        1. Wait for service and action server are successful.
    #        2. Goal is not executed.
    #        3. Service call is successful.
    #        4. Goal is executed.
    #        5. Goal execution is stopped. The robot is led into a hold position, but not stopped abruptly.
    def runTest(self):

        rospy.loginfo("1. Set up hold service and trajectory dispatcher.")
        action_name = controller_ns + ACTION_NAME

        hold_srv = StopServiceWrapper()
        trajectory_dispatcher = TrajectoryDispatcher()
        is_executing_srv = IsExecutingServiceWrapper()

        rospy.loginfo("2. Send goal to controller action server. Default startup state should be holding!!!!")
        trajectory_dispatcher.dispatchTrajectory(goal_position = DEFAULT_GOAL_POSITION, time_from_start = 1)
        result = trajectory_dispatcher.waitForResult()
        self.assertEqual(FollowJointTrajectoryResult.INVALID_GOAL, result.error_code,
                         'Unexpected error code in result of ' + action_name + ' action: ' + str(result.error_code))

        rospy.loginfo("3. Request switch to DEFAULT mode via service.")
        self.assertTrue(hold_srv.requestDefaultMode(), 'Switch to Mode DEFAULT failed.')

        rospy.loginfo("4. Send goal to controller action server.")
        trajectory_dispatcher.dispatchTrajectory(goal_position = DEFAULT_GOAL_POSITION, time_from_start = 1)
        result = trajectory_dispatcher.waitForResult()
        self.assertEqual(FollowJointTrajectoryResult.SUCCESSFUL, result.error_code, 'Action goal was not successful.')

        rospy.loginfo("5. Send new goal to controller action server and switch to HOLDING during execution.")
        trajectory_dispatcher.dispatchTrajectory(goal_position = 0.2, time_from_start = 5)

        # Wait for movement to commence
        MovementObserver().observeUntilPositionGreater(0.11, 3)
        self.assertTrue(is_executing_srv.call(), 'is_executing service returned false.')
        # Hold robot during movement
        rospy.loginfo("  - Robot is moving -> Switch into HOLDING mode")
        self.assertTrue(hold_srv.requestHoldingMode(), 'Switch to Mode HOLDING failed.')

        # Make sure robot stops (not abruptly)
        MovementObserver().observeStop()
        rospy.loginfo("  - Stop of robot observed!")

        result = trajectory_dispatcher.waitForResult()
        self.assertEqual(FollowJointTrajectoryResult.SUCCESSFUL, result.error_code, 'Action goal was not successful.')

        # Make sure goal was preempted
        self.assertEqual(GoalStatus.PREEMPTED, trajectory_dispatcher.getLastState(), 'Action goal was not preempted.')

if __name__ == '__main__':
    import rostest
    rospy.init_node('integrationtest_pilz_joint_trajectory_controller')
    rostest.rosrun(PACKAGE_NAME, 'integrationtest_pilz_joint_trajectory_controller',
                   IntegrationtestPilzJointTrajectoryController)
