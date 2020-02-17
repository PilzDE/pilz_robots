/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <pilz_testutils/async_test.h>
#include <pilz_testutils/logger_mock.h>

#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Trigger.h>

#include <hardware_interface/joint_command_interface.h>
#include <trajectory_interface/quintic_spline_segment.h>

#include <pilz_control/pilz_joint_trajectory_controller.h>
#include <pilz_control/pilz_joint_trajectory_controller_impl.h>

#include "pjtc_manager_mock.h"
#include "pjtc_test_helper.h"
#include "robot_driver_mock.h"
#include "trajectory_action_client_wrapper.h"

namespace pilz_joint_trajectory_controller_test
{
static const std::string CONTROLLER_NAMESPACE{ "/controller_ns" };
static const std::string TRAJECTORY_ACTION{ "/follow_joint_trajectory" };
static const std::string HOLD_SERVICE{ "/hold" };
static const std::string UNHOLD_SERVICE{ "/unhold" };
static const std::string IS_EXECUTING_SERVICE{ "/is_executing" };
static const std::string TRAJECTORY_COMMAND_TOPIC{ "/command" };

using HWInterface = hardware_interface::PositionJointInterface;
using Segment = trajectory_interface::QuinticSplineSegment<double>;
using PJTCManager = PJTCManagerMock<Segment, HWInterface>;
using RobotDriver = RobotDriverMock<PJTCManager>;
using Controller = pilz_joint_trajectory_controller::PilzJointTrajectoryController<Segment, HWInterface>;
using ControllerPtr = std::shared_ptr<Controller>;

/**
 * @brief Test fixture class for the unit-test of the PilzJointTrajectoryController.
 *
 * A minimal test-robot is used to initialize the controller.
 *
 * @note ros::Time is set to use simulated time.
 */
class PilzJointTrajectoryControllerTest : public testing::Test, public testing::AsyncTest
{
protected:
  void SetUp() override;

  testing::AssertionResult isControllerInHoldMode();
  testing::AssertionResult isControllerInUnholdMode();

protected:
  RobotDriver robot_driver_{ CONTROLLER_NAMESPACE };
  std::shared_ptr<PJTCManager> manager_;
  TrajectoryActionClientWrapper action_client_{ CONTROLLER_NAMESPACE + TRAJECTORY_ACTION };
  ros::AsyncSpinner spinner_{ 2 };
};

void PilzJointTrajectoryControllerTest::SetUp()
{
  spinner_.start();
  manager_ = robot_driver_.getManager();
  setControllerParameters(CONTROLLER_NAMESPACE);
  startSimTime();
}

testing::AssertionResult PilzJointTrajectoryControllerTest::isControllerInHoldMode()
{
  GoalType goal{ generateAlternatingGoal<RobotDriver>(&robot_driver_) };
  action_client_.sendGoal(goal);
  if (!action_client_.waitForActionResult())
  {
    return testing::AssertionFailure() << "Failed to get result after sending goal in hold mode.";
  }
  if (action_client_.getResult()->error_code != control_msgs::FollowJointTrajectoryResult::INVALID_GOAL)
  {
    return testing::AssertionFailure() << "Error code is " << action_client_.getResult()->error_code
                                       << ", should have been INVALID_GOAL";
  }
  return testing::AssertionSuccess();
}

testing::AssertionResult PilzJointTrajectoryControllerTest::isControllerInUnholdMode()
{
  GoalType goal{ generateAlternatingGoal<RobotDriver>(&robot_driver_) };
  action_client_.sendGoal(goal);
  if (!action_client_.waitForActionResult([this]() { robot_driver_.update(); }))
  {
    return testing::AssertionFailure() << "Failed to get result after sending goal in unhold mode.";
  }
  if (action_client_.getResult()->error_code != control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
  {
    return testing::AssertionFailure() << "Error code is " << action_client_.getResult()->error_code
                                       << ", should have been SUCCESSFUL";
  }
  return testing::AssertionSuccess();
}

//////////////////////////////////
//  Testing of Initialization   //
//////////////////////////////////

TEST_F(PilzJointTrajectoryControllerTest, testInitializiation)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";

  EXPECT_TRUE(ros::service::exists(CONTROLLER_NAMESPACE + HOLD_SERVICE, true));
  EXPECT_TRUE(ros::service::exists(CONTROLLER_NAMESPACE + UNHOLD_SERVICE, true));
  EXPECT_TRUE(ros::service::exists(CONTROLLER_NAMESPACE + IS_EXECUTING_SERVICE, true));

  EXPECT_TRUE(action_client_.waitForActionServer());
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants are called.
 */
TEST_F(PilzJointTrajectoryControllerTest, testD0Destructor)
{
  ControllerPtr controller{ new Controller() };
  SUCCEED();
}

/////////////////////////////////////
//    Testing of hold and unhold   //
/////////////////////////////////////

/**
 * @tests{end_holding,
 * Tests unholding before the controller is started.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testUnholdFailureWhenNotStarted)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  EXPECT_TRUE(manager_->triggerUnHold(req, resp));
  EXPECT_FALSE(resp.success);
}

/**
 * @tests{hold_at_controller_start,
 * Tests that the controller is holded once it is started.
 * }
 * @tests{no_execution_during_hold,
 * Tests that the controller is holded once it is started.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testHoldAtStart)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";
  ASSERT_TRUE(action_client_.waitForActionServer());

  manager_->startController();

  EXPECT_TRUE(isControllerInHoldMode());
}

/**
 * @tests{end_holding,
 * Tests unholding the controller at different time points after the controller is started.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testUnholdSuccess)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";
  ASSERT_TRUE(action_client_.waitForActionServer());

  manager_->startController();

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  EXPECT_TRUE(manager_->triggerUnHold(req, resp));
  EXPECT_FALSE(resp.success);

  ros::Duration stop_duration{ STOP_TRAJECTORY_DURATION_SEC + 2 * DEFAULT_UPDATE_PERIOD_SEC };
  progressInTime(stop_duration);
  robot_driver_.update();

  EXPECT_TRUE(manager_->triggerUnHold(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInUnholdMode());
}

/**
 * @tests{start_holding,
 * Tests holding the controller.
 * }
 * @tests{no_execution_during_hold,
 * Tests holding the controller.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testHoldSuccessAfterUnhold)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = manager_->triggerHoldAsync(req, resp);
  EXPECT_TRUE(updateUntilHoldMode<RobotDriver>(&robot_driver_, hold_future));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInHoldMode());
}

/**
 * @tests{start_holding,
 * Tests triggering hold two times successively.
 * }
 * @tests{no_execution_during_hold,
 * Tests triggering hold two times successively.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testDoubleHoldSuccess)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = manager_->triggerHoldAsync(req, resp);

  EXPECT_TRUE(updateUntilHoldMode<RobotDriver>(&robot_driver_, hold_future));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInHoldMode());

  EXPECT_TRUE(manager_->triggerHold(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInHoldMode());
}

/**
 * @tests{end_holding,
 * Tests triggering unhold two times successively.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testDoubleUnholdSuccess)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;

  EXPECT_TRUE(manager_->triggerUnHold(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInUnholdMode());

  EXPECT_TRUE(manager_->triggerUnHold(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInUnholdMode());
}

/**
 * @tests{end_holding,
 * Tests holding and unholding the controller repeatedly.
 * }
 * @tests{start_holding,
 * Tests holding and unholding the controller repeatedly.
 * }
 * @tests{no_execution_during_hold,
 * Tests holding and unholding the controller repeatedly.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testRepeatHoldAndUnholdSuccess)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  const unsigned int number_of_iterations{ 3U };
  for (unsigned int i = 0; i < number_of_iterations; ++i)
  {
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse resp;
    // run async such that stop trajectory can be executed in the meantime
    std::future<bool> hold_future = manager_->triggerHoldAsync(req, resp);
    EXPECT_TRUE(updateUntilHoldMode<RobotDriver>(&robot_driver_, hold_future));
    EXPECT_TRUE(resp.success);
    EXPECT_TRUE(isControllerInHoldMode());

    EXPECT_TRUE(manager_->triggerUnHold(req, resp));
    EXPECT_TRUE(resp.success);
    EXPECT_TRUE(isControllerInUnholdMode());
  }
}

/**
 * @tests{start_holding,
 * Tests holding the controller with a running trajectory.
 * }
 * @tests{no_execution_during_hold,
 * Tests holding the controller with a running trajectory.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testHoldDuringGoalExecution)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  GoalType goal{ generateAlternatingGoal<RobotDriver>(&robot_driver_) };
  action_client_.sendGoal(goal);

  EXPECT_TRUE(updateUntilRobotMotion<RobotDriver>(&robot_driver_));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = manager_->triggerHoldAsync(req, resp);
  EXPECT_TRUE(updateUntilHoldMode<RobotDriver>(&robot_driver_, hold_future));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInHoldMode());

  EXPECT_TRUE(action_client_.waitForActionResult());
  EXPECT_EQ(action_client_.getResult()->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_GOAL);
}

/**
 * @brief Cancel a goal after a hold was requested and before the next update is performed.
 *
 * This test was written to obtain full line coverage.
 */
TEST_F(PilzJointTrajectoryControllerTest, testGoalCancellingDuringHold)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  GoalType goal{ generateAlternatingGoal<RobotDriver>(&robot_driver_) };
  action_client_.sendGoal(goal);

  EXPECT_TRUE(updateUntilRobotMotion<RobotDriver>(&robot_driver_));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = manager_->triggerHoldAsync(req, resp);

  // Sleep to make sure hold was triggered
  std::this_thread::sleep_for(HOLD_TIMEOUT);
  action_client_.cancelGoal();
  EXPECT_TRUE(action_client_.waitForActionResult());
  robot_driver_.update();
  EXPECT_TRUE(updateUntilHoldMode<RobotDriver>(&robot_driver_, hold_future));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(isControllerInHoldMode());
}

using pilz_testutils::IsINFO;
using pilz_testutils::IsWARN;
using ::testing::_;
using ::testing::InSequence;
static const std::string LOG_MSG_RECEIVED_EVENT_WARN{ "log_msg_received_warn" };
static const std::string LOG_MSG_RECEIVED_EVENT_INFO{ "log_msg_received_info" };
/**
 * @brief Tests that the controller does not execute if a trajectory is sent via command interface.
 */
TEST_F(PilzJointTrajectoryControllerTest, testTrajCommandExecution)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  ros::NodeHandle nh{ "~" };
  ros::Publisher trajectory_command_publisher =
      nh.advertise<trajectory_msgs::JointTrajectory>(CONTROLLER_NAMESPACE + TRAJECTORY_COMMAND_TOPIC, 1);

  GoalType goal{ generateAlternatingGoal(&robot_driver_) };

  pilz_testutils::LoggerMock logger_mock_holder;
  InSequence seq;
  EXPECT_LOG(*logger_mock_holder, WARN,
             pilz_joint_trajectory_controller::USER_NOTIFICATION_NOT_IMPLEMENTED_COMMAND_INTERFACE_WARN)
      .WillOnce(ACTION_OPEN_BARRIER_VOID(LOG_MSG_RECEIVED_EVENT_WARN));
  EXPECT_LOG(*logger_mock_holder, INFO,
             pilz_joint_trajectory_controller::USER_NOTIFICATION_NOT_IMPLEMENTED_COMMAND_INTERFACE_INFO)
      .WillOnce(ACTION_OPEN_BARRIER_VOID(LOG_MSG_RECEIVED_EVENT_INFO));

  trajectory_command_publisher.publish(goal.trajectory);

  ros::Duration observation_time = getGoalDuration(goal) + ros::Duration(DEFAULT_UPDATE_PERIOD_SEC);
  EXPECT_FALSE(updateUntilRobotMotion(&robot_driver_));  // motion would not start before goal message is received
  progressInTime(observation_time);
  EXPECT_FALSE(updateUntilRobotMotion(&robot_driver_));  // at this time the goal would have been reached
  BARRIER({ LOG_MSG_RECEIVED_EVENT_WARN, LOG_MSG_RECEIVED_EVENT_INFO });
}

/////////////////////////////////////////////////////////////////////////////////////////////
//    Testing the correct handling of trajectories that violate the acceleration limits    //
/////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Send a trajectory that has too high acceleration and make sure controller does not execute it.
 */
TEST_F(PilzJointTrajectoryControllerTest, testTrajectoryWithTooHighAcceleration)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  // Make sure robot moves for slow motion
  GoalType goal{ generateAlternatingGoal<RobotDriver>(&robot_driver_) };
  action_client_.sendGoal(goal);

  EXPECT_TRUE(updateUntilRobotMotion(&robot_driver_));
  EXPECT_TRUE(updateUntilNoRobotMotion(&robot_driver_));

  action_client_.waitForActionResult();
  EXPECT_EQ(action_client_.getResult()->error_code, control_msgs::FollowJointTrajectoryResult::SUCCESSFUL);

  // Now sending a quicker motion which should trigger the acceleration limit and not move the robot
  const auto start_position = robot_driver_.getJointPositions();

  goal = generateAlternatingGoal(&robot_driver_, ros::Duration(DEFAULT_GOAL_DURATION_SEC), 1E3);
  action_client_.sendGoal(goal);
  action_client_.waitForActionResult([this]() { robot_driver_.update(); });
  // the following fails due to https://github.com/ros-controls/ros_controllers/issues/174
  // EXPECT_EQ(action_client_.getResult()->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_GOAL);

  const auto end_position = robot_driver_.getJointPositions();
  ASSERT_EQ(end_position.size(), start_position.size()) << "Position vectors sizes mismatch.";
  for (unsigned int i = 0; i < end_position.size(); ++i)
  {
    EXPECT_EQ(end_position[i], start_position[i])
        << "Difference in position despite no movement should have been performed.";
  }
}

}  // namespace pilz_joint_trajectory_controller_test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_pilz_joint_trajectory_controller");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
