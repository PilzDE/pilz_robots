/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
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

#include <functional>
#include <future>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <ros/ros.h>

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

using HWInterface = hardware_interface::PositionJointInterface;
using Segment = trajectory_interface::QuinticSplineSegment<double>;
using PJTCManager = PJTCManagerMock<Segment, HWInterface>;
using RobotDriver = RobotDriverMock<PJTCManager>;
using Controller = pilz_joint_trajectory_controller::PilzJointTrajectoryController<Segment, HWInterface>;
using ControllerPtr = std::shared_ptr<Controller>;

//! The return value indicates if the call was successful (in case of a service callback), the actual result
//! of the is-executing-check is assigned (via reference) to the second argument.
using InvokeIsExecuting = std::function<testing::AssertionResult(const ControllerPtr&, bool&)>;

static testing::AssertionResult InvokeIsExecutingMethod(const ControllerPtr& controller, bool& result)
{
  result = controller->is_executing();
  return testing::AssertionSuccess();
}

static testing::AssertionResult InvokeIsExecutingServiceCallback(const ControllerPtr& controller, bool& result)
{
  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  if (!controller->handleIsExecutingRequest(req, resp))
  {
    return testing::AssertionFailure() << "Callback of is_executing returned false unexpectedly.";
  }

  result = resp.success;
  return testing::AssertionSuccess();
}

/**
 * @brief For testing both the isExecuting method and the is_executing service callback we use parameterized tests.
 */
class PilzJointTrajectoryControllerIsExecutingTest : public testing::Test,
                                                     public testing::WithParamInterface<InvokeIsExecuting>
{
protected:
  void SetUp() override;

  testing::AssertionResult invokeIsExecuting(bool& result);

protected:
  RobotDriver robot_driver_{ CONTROLLER_NAMESPACE };
  std::shared_ptr<PJTCManager> manager_;
  TrajectoryActionClientWrapper action_client_{ CONTROLLER_NAMESPACE + TRAJECTORY_ACTION };
  ros::AsyncSpinner spinner_{ 2 };
};

void PilzJointTrajectoryControllerIsExecutingTest::SetUp()
{
  spinner_.start();
  manager_ = robot_driver_.getManager();
  setControllerParameters(CONTROLLER_NAMESPACE);
  startSimTime();
}

testing::AssertionResult PilzJointTrajectoryControllerIsExecutingTest::invokeIsExecuting(bool& result)
{
  auto invoke_is_executing{ GetParam() };
  return invoke_is_executing(manager_->controller_, result);
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testNotStarted)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

/**
 * @brief Check that a fake start (simply setting state to RUNNING) doesn't trigger that is_executing() returns true.
 *
 * Increases line coverage.
 */
TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testFakeStart)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";

  manager_->controller_->state_ = controller_interface::ControllerBase::ControllerState::RUNNING;
  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testStopTrajExecutionAtStart)
{
  ASSERT_TRUE(manager_->loadController()) << "Failed to initialize the controller.";

  manager_->startController();

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_TRUE(is_executing_result) << "Failed to detect stop trajectory execution at start.";

  ros::Duration stop_duration{ STOP_TRAJECTORY_DURATION_SEC + 2 * DEFAULT_UPDATE_PERIOD_SEC };
  progressInTime(stop_duration);
  robot_driver_.update();

  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testNotExecutingAfterUnhold)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testActionGoalExecution)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  GoalType goal{ generateAlternatingGoal<RobotDriver>(&robot_driver_) };
  action_client_.sendGoal(goal);

  EXPECT_TRUE(updateUntilRobotMotion<RobotDriver>(&robot_driver_));

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_TRUE(is_executing_result) << "Failed to detect action goal execution";

  progressInTime(getGoalDuration(goal) + ros::Duration(DEFAULT_UPDATE_PERIOD_SEC));
  robot_driver_.update();

  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testStopTrajExecutionAtHold)
{
  ASSERT_TRUE(performFullControllerStartup(&robot_driver_));

  GoalType goal{ generateAlternatingGoal<RobotDriver>(&robot_driver_) };
  action_client_.sendGoal(goal);

  EXPECT_TRUE(updateUntilRobotMotion<RobotDriver>(&robot_driver_));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = manager_->triggerHoldAsync(req, resp);

  EXPECT_TRUE(action_client_.waitForActionResult());
  // the following fails due to https://github.com/ros-controls/ros_controllers/issues/174
  // EXPECT_EQ(action_client_.getResult()->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_GOAL);

  robot_driver_.update();

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_TRUE(is_executing_result) << "Failed to detect stop trajectory execution";

  EXPECT_TRUE(updateUntilHoldMode<RobotDriver>(&robot_driver_, hold_future));
  EXPECT_TRUE(resp.success);

  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

INSTANTIATE_TEST_CASE_P(MethodAndServiceCallback, PilzJointTrajectoryControllerIsExecutingTest,
                        testing::Values(InvokeIsExecutingMethod, InvokeIsExecutingServiceCallback));

}  // namespace pilz_joint_trajectory_controller_test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_pilz_joint_trajectory_controller_is_executing");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
