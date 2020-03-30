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

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <thread>
#include <unistd.h>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <controller_interface/controller_base.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <pilz_control/pilz_joint_trajectory_controller.h>

typedef hardware_interface::PositionJointInterface HWInterface;
typedef trajectory_interface::QuinticSplineSegment<double> Segment;
typedef pilz_joint_trajectory_controller::PilzJointTrajectoryController<Segment, HWInterface> Controller;
typedef std::shared_ptr<Controller> ControllerPtr;
typedef control_msgs::FollowJointTrajectoryGoal GoalType;

namespace pilz_joint_trajectory_controller
{

static const std::string CONTROLLER_NAMESPACE {"/controller_ns"};
static const std::string TRAJECTORY_ACTION {"/follow_joint_trajectory"};
static const std::string HOLD_SERVICE {"/hold"};
static const std::string UNHOLD_SERVICE {"/unhold"};
static const std::string IS_EXECUTING_SERVICE {"/is_executing"};
static const std::string MONITOR_CARTESIAN_SPEED_SERVICE {"/monitor_cartesian_speed"};
static const std::string TRAJECTORY_COMMAND_TOPIC {"/command"};

static const std::string STOP_TRAJECTORY_DURATION_PARAMETER {"stop_trajectory_duration"};
static const std::string GOAL_TIME_TOLERANCE_PARAMETER {"constraints/goal_time"};
static const std::string JOINTS_PARAMETER {"joints"};
static const std::vector<std::string> JOINT_NAMES {"shoulder_to_right_arm", "shoulder_to_left_arm"};

static constexpr double DEFAULT_GOAL_DURATION_SEC {1.0};
static constexpr double DEFAULT_UPDATE_PERIOD_SEC {0.008};
static constexpr double STOP_TRAJECTORY_DURATION_SEC {0.2};
static constexpr double GOAL_TIME_TOLERANCE_SEC {0.01};
static constexpr double TIME_SIMULATION_START_SEC {0.1};
static constexpr double VELOCITY_COMPARISON_EPS {0.01};
static constexpr double WAIT_FOR_ACTION_RESULT_TIMEOUT_SEC {5.0};

static constexpr unsigned int SLEEP_TIME_MSEC {5};
static constexpr unsigned int WAIT_FOR_ACTION_SERVER_TIMEOUT_MSEC {5000};
static constexpr unsigned int WAIT_FOR_HOLD_FUTURE_MSEC {1000};
static constexpr unsigned int WAIT_FOR_MOVEMENT_STARTED_MSEC {1000};

static void progressInTime(const ros::Duration& period)
{
  ros::Time current_time {ros::Time::now()};
  ros::Time::setNow(current_time + period);
}

static ros::Duration getGoalDuration(const GoalType &goal)
{
  return goal.trajectory.points.back().time_from_start;
}

static bool waitForUsingSystemTime(const std::function<bool()>& is_condition_fulfilled,
                                   const std::chrono::milliseconds& timeout)
{
  const std::chrono::system_clock::time_point start {std::chrono::system_clock::now()};
  do
  {
    if (is_condition_fulfilled())
    {
      return true;
    }
    if (std::chrono::system_clock::now() - start > timeout)
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_MSEC));
  }
  while (ros::ok());
  return false;
}

template<typename T>
bool isFutureReady(const std::future<T>& this_future)
{
  return this_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

/**
 * @brief For simplicity this robot only updates one joint of the two joints defined in the urdf.
 */
class RobotMock
{
public:
  RobotMock(HWInterface* hardware);

  void update(const ros::Duration& period);

  bool isMoving();

private:
  double pos_ = 0.0, vel_ = 0.0, eff_ = 0.0, cmd_ = 0.0;
  double pos2_ = 0.0, vel2_ = 0.0, eff2_ = 0.0, cmd2_ = 0.0;
};

RobotMock::RobotMock(HWInterface* hardware)
{
  hardware_interface::JointStateHandle jsh {JOINT_NAMES.at(0), &pos_, &vel_, &eff_};
  hardware_interface::JointHandle jh {jsh, &cmd_};
  hardware->registerHandle(jh);
  hardware_interface::JointStateHandle jsh2 {JOINT_NAMES.at(1), &pos2_, &vel2_, &eff2_};
  hardware_interface::JointHandle jh2 {jsh2, &cmd2_};
  hardware->registerHandle(jh2);
}

void RobotMock::update(const ros::Duration& period)
{
vel_ = (cmd_ - pos_) / period.toSec();
pos_ = cmd_;
}

bool RobotMock::isMoving()
{
  return std::abs(vel_) > VELOCITY_COMPARISON_EPS;
}

/**
 * @brief Test fixture class for the unit-test of the PilzJointTrajectoryController.
 *
 * A minimal test-robot is used to initialize the controller.
 *
 * @note ros::Time is set to use simulated time.
 */
class PilzJointTrajectoryControllerTest : public testing::Test
{
protected:
  void SetUp() override;

  void startController();

  void updateController();

  bool updateControllerUntil(const std::function<bool()>& is_condition_fulfilled,
                             const std::chrono::milliseconds& timeout);

  bool waitForActionServer(const std::chrono::milliseconds& timeout
                           = std::chrono::milliseconds(WAIT_FOR_ACTION_SERVER_TIMEOUT_MSEC));

  bool waitForActionResult(bool perform_controller_updates = false,
                           const ros::Duration& timeout = ros::Duration(WAIT_FOR_ACTION_RESULT_TIMEOUT_SEC));

  GoalType generateSimpleGoal(const ros::Duration& goal_duration = ros::Duration(DEFAULT_GOAL_DURATION_SEC));

  testing::AssertionResult checkForHold();

  testing::AssertionResult checkForUnhold();

  /**
   * @brief Perform init, start, unhold and update, such that controller is ready for executing.
   */
  testing::AssertionResult performFullControllerStartup();

protected:
  ControllerPtr controller_;
  HWInterface* hardware_ {new HWInterface()};
  std::unique_ptr<RobotMock> robot_;
  ros::NodeHandle nh_ {"~"};
  ros::NodeHandle controller_nh_ {CONTROLLER_NAMESPACE};
  ros::AsyncSpinner spinner_ {2};
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
        trajectory_action_client_ {CONTROLLER_NAMESPACE + TRAJECTORY_ACTION, true};
  std::shared_ptr<ros::Publisher> trajectory_command_publisher_;
  ros::Time last_update_time_ {TIME_SIMULATION_START_SEC};
};

void PilzJointTrajectoryControllerTest::SetUp()
{
  spinner_.start();

  robot_.reset(new RobotMock(hardware_));

  // set joints parameter
  controller_nh_.setParam(JOINTS_PARAMETER, JOINT_NAMES);

  // Setup controller
  controller_ = std::make_shared<Controller>();

  trajectory_command_publisher_ = std::make_shared<ros::Publisher>(
          nh_.advertise<trajectory_msgs::JointTrajectory>(CONTROLLER_NAMESPACE + TRAJECTORY_COMMAND_TOPIC, 1));

  // Set stop trajectory duration on parameter server (will be read in controller_->init())
  controller_nh_.setParam(STOP_TRAJECTORY_DURATION_PARAMETER, STOP_TRAJECTORY_DURATION_SEC);

  controller_nh_.setParam(GOAL_TIME_TOLERANCE_PARAMETER, GOAL_TIME_TOLERANCE_SEC);

  // Set up simulated time
  ros::Time start_time {TIME_SIMULATION_START_SEC};
  ros::Time::setNow(start_time);
}

void PilzJointTrajectoryControllerTest::startController()
{
  ros::Time current_time {ros::Time::now()};
  controller_->starting(current_time);
  last_update_time_ = current_time;
}

void PilzJointTrajectoryControllerTest::updateController()
{
  ros::Time current_time {ros::Time::now()};
  controller_->update(current_time, current_time - last_update_time_);
  robot_->update(current_time - last_update_time_);
  last_update_time_ = current_time;
}


bool PilzJointTrajectoryControllerTest::updateControllerUntil(const std::function<bool()>& is_condition_fulfilled,
                                                              const std::chrono::milliseconds& timeout)
{
  const std::chrono::system_clock::time_point start {std::chrono::system_clock::now()};
  do
  {
    if (is_condition_fulfilled())
    {
      return true;
    }
    if (std::chrono::system_clock::now() - start > timeout)
    {
      return false;
    }
    progressInTime(ros::Duration(DEFAULT_UPDATE_PERIOD_SEC));
    updateController();
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_MSEC));
  }
  while (ros::ok());
  return false;
}

bool PilzJointTrajectoryControllerTest::waitForActionServer(const std::chrono::milliseconds& timeout)
{
  return waitForUsingSystemTime([this](){ return trajectory_action_client_.isServerConnected(); }, timeout);
}

bool PilzJointTrajectoryControllerTest::waitForActionResult(bool perform_controller_updates,
                                                            const ros::Duration& timeout)
{
  std::future<bool> wait_for_result_future = std::async(std::launch::async,
                                                        [this, &timeout]()
                                                        {
                                                          return trajectory_action_client_.waitForResult(timeout);
                                                        });
  while (!isFutureReady(wait_for_result_future))
  {
    progressInTime(ros::Duration(DEFAULT_UPDATE_PERIOD_SEC));
    if (perform_controller_updates)
    {
      updateController();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_MSEC));
  }
  return wait_for_result_future.get();
}

GoalType PilzJointTrajectoryControllerTest::generateSimpleGoal(const ros::Duration &goal_duration)
{
  static unsigned int position_index {0};
  const std::vector<double> alternating_positions {0.5, -0.25, -0.5};
  position_index = (++position_index) % alternating_positions.size();

  GoalType goal;
  goal.trajectory.joint_names = JOINT_NAMES;
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].time_from_start = goal_duration;
  goal.trajectory.points[0].positions = {alternating_positions[position_index], 0.0};

  return goal;
}

testing::AssertionResult PilzJointTrajectoryControllerTest::checkForHold()
{
  GoalType goal {generateSimpleGoal()};
  trajectory_action_client_.sendGoal(goal);
  if (!waitForActionResult())
  {
    return testing::AssertionFailure() << "Failed to get result after sending goal in hold mode.";
  }
  if (trajectory_action_client_.getResult()->error_code != control_msgs::FollowJointTrajectoryResult::INVALID_GOAL)
  {
    return testing::AssertionFailure() << "Error code is " << trajectory_action_client_.getResult()->error_code
                                       << ", should have been INVALID_GOAL";
  }
  return testing::AssertionSuccess();
}

testing::AssertionResult PilzJointTrajectoryControllerTest::checkForUnhold()
{
  GoalType goal {generateSimpleGoal()};
  trajectory_action_client_.sendGoal(goal);
  if (!waitForActionResult(true))
  {
    return testing::AssertionFailure() << "Failed to get result after sending goal in unhold mode.";
  }
  if (trajectory_action_client_.getResult()->error_code != control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
  {
    return testing::AssertionFailure() << "Error code is " << trajectory_action_client_.getResult()->error_code
                                       << ", should have been SUCCESSFUL";
  }
  return testing::AssertionSuccess();
}

testing::AssertionResult PilzJointTrajectoryControllerTest::performFullControllerStartup()
{
  if (!controller_->init(hardware_, nh_, controller_nh_))
  {
    return testing::AssertionFailure() << "Failed to initialize the controller.";
  }
  if (!waitForActionServer())
  {
    return testing::AssertionFailure() << "Failed to connect to action server.";
  }
  controller_->state_ = controller_->INITIALIZED;

  startController();
  controller_->state_ = controller_->RUNNING;

  ros::Duration stop_duration{STOP_TRAJECTORY_DURATION_SEC + 2*DEFAULT_UPDATE_PERIOD_SEC};
  progressInTime(stop_duration);
  updateController();

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  if (!(controller_->handleUnHoldRequest(req, resp) && resp.success))
  {
    return testing::AssertionFailure() << "Unholding the controller failed.";
  }

  return testing::AssertionSuccess();
}


///////////////////////////////////////
//    The actual tests start here    //
///////////////////////////////////////

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants are called.
 */
TEST_F(PilzJointTrajectoryControllerTest, testD0Destructor)
{
  ControllerPtr controller {new Controller()};
  SUCCEED();
}

TEST_F(PilzJointTrajectoryControllerTest, testInitializiation)
{
  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";

  ASSERT_TRUE(ros::service::exists(controller_nh_.getNamespace() + HOLD_SERVICE, true));
  ASSERT_TRUE(ros::service::exists(controller_nh_.getNamespace() + UNHOLD_SERVICE, true));
  EXPECT_TRUE(ros::service::exists(controller_nh_.getNamespace() + IS_EXECUTING_SERVICE, true));

  EXPECT_TRUE(waitForActionServer());
}

/**
 * @tests{end_holding,
 * Tests unholding before the controller is started.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testUnholdFailureWhenNotStarted)
{
  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";
  controller_->state_ = controller_->INITIALIZED;

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  EXPECT_TRUE(controller_->handleUnHoldRequest(req, resp));
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
  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";
  ASSERT_TRUE(waitForActionServer());
  controller_->state_ = controller_->INITIALIZED;

  startController();
  controller_->state_ = controller_->RUNNING;

  EXPECT_TRUE(checkForHold());
}

/**
 * @tests{end_holding,
 * Tests unholding the controller.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testUnholdSuccess)
{
  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";
  ASSERT_TRUE(waitForActionServer());
  controller_->state_ = controller_->INITIALIZED;

  startController();
  controller_->state_ = controller_->RUNNING;

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  EXPECT_TRUE(controller_->handleUnHoldRequest(req, resp));
  EXPECT_FALSE(resp.success);

  ros::Duration stop_duration{STOP_TRAJECTORY_DURATION_SEC + 2*DEFAULT_UPDATE_PERIOD_SEC};
  progressInTime(stop_duration);
  updateController();

  EXPECT_TRUE(controller_->handleUnHoldRequest(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(checkForUnhold());
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
  ASSERT_TRUE(performFullControllerStartup());

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = std::async(std::launch::async,
                                             [this, &req, &resp]()
                                             {
                                               return controller_->handleHoldRequest(req, resp);
                                             });

  std::chrono::milliseconds hold_timeout {WAIT_FOR_HOLD_FUTURE_MSEC};
  EXPECT_TRUE(updateControllerUntil([&hold_future](){ return isFutureReady(hold_future); }, hold_timeout));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(checkForHold());
}

/**
 * @tests{start_holding,
 * Tests holding the controller.
 * }
 * @tests{no_execution_during_hold,
 * Tests holding the controller.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testDoubleHoldSuccess)
{
  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";
  ASSERT_TRUE(waitForActionServer());
  controller_->state_ = controller_->INITIALIZED;

  startController();
  controller_->state_ = controller_->RUNNING;

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = std::async(std::launch::async,
                                             [this, &req, &resp]()
                                             {
                                               return controller_->handleHoldRequest(req, resp);
                                             });

  std::chrono::milliseconds hold_timeout {WAIT_FOR_HOLD_FUTURE_MSEC};
  EXPECT_TRUE(updateControllerUntil([&hold_future](){ return isFutureReady(hold_future); }, hold_timeout));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(checkForHold());

  EXPECT_TRUE(controller_->handleHoldRequest(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(checkForHold());
}

/**
 * @tests{end_holding,
 * Tests unholding the controller.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testDoubleUnholdSuccess)
{
  ASSERT_TRUE(performFullControllerStartup());

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;

  EXPECT_TRUE(controller_->handleUnHoldRequest(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(checkForUnhold());
}

/**
 * @tests{end_holding,
 * Tests unholding the controller.
 * }
 * @tests{start_holding,
 * Tests holding the controller.
 * }
 * @tests{no_execution_during_hold,
 * Tests holding the controller.
 * }
 */
TEST_F(PilzJointTrajectoryControllerTest, testRepeatHoldAndUnholdSuccess)
{
  ASSERT_TRUE(performFullControllerStartup());

  const unsigned int number_of_iterations {3U};
  for (unsigned int i = 0; i < number_of_iterations; ++i)
  {
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse resp;
    // run async such that stop trajectory can be executed in the meantime
    std::future<bool> hold_future = std::async(std::launch::async,
                                               [this, &req, &resp]()
                                               {
                                                 return controller_->handleHoldRequest(req, resp);
                                               });

    std::chrono::milliseconds hold_timeout {WAIT_FOR_HOLD_FUTURE_MSEC};
    EXPECT_TRUE(updateControllerUntil([&hold_future](){ return isFutureReady(hold_future); }, hold_timeout));
    EXPECT_TRUE(resp.success);
    EXPECT_TRUE(checkForHold());

    EXPECT_TRUE(controller_->handleUnHoldRequest(req, resp));
    EXPECT_TRUE(resp.success);
    EXPECT_TRUE(checkForUnhold());
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
  ASSERT_TRUE(performFullControllerStartup());

  GoalType goal {generateSimpleGoal()};
  trajectory_action_client_.sendGoal(goal);

  std::chrono::milliseconds movement_timeout {WAIT_FOR_MOVEMENT_STARTED_MSEC};
  EXPECT_TRUE(updateControllerUntil([this](){ return robot_->isMoving(); }, movement_timeout));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = std::async(std::launch::async,
                                             [this, &req, &resp]()
                                             {
                                               return controller_->handleHoldRequest(req, resp);
                                             });

  std::chrono::milliseconds hold_timeout {WAIT_FOR_HOLD_FUTURE_MSEC};
  EXPECT_TRUE(updateControllerUntil([&hold_future](){ return isFutureReady(hold_future); }, hold_timeout));
  EXPECT_TRUE(resp.success);
  EXPECT_TRUE(checkForHold());

  EXPECT_TRUE(waitForActionResult());
  EXPECT_EQ(trajectory_action_client_.getResult()->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_GOAL);
}


//////////////////////////////////////////////////////////////////////////
//    Parameterized tests for the "is-executing-check" functionality    //
//////////////////////////////////////////////////////////////////////////

//! The return value indicates if the call was successful (in case of a service callback), the actual result
//! of the is-executing-check is assigned (via reference) to the second argument.
typedef std::function<testing::AssertionResult(const ControllerPtr&, bool&)> InvokeIsExecuting;

static testing::AssertionResult InvokeIsExecutingMethod(const ControllerPtr& controller, bool& result)
{
  result = controller->is_executing();
  return testing::AssertionSuccess();
}

static testing::AssertionResult InvokeIsExecutingServiceCallback(const ControllerPtr& controller, bool &result)
{
  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  if(!controller->handleIsExecutingRequest(req, resp))
  {
    return testing::AssertionFailure() << "Callback of is_executing returned false unexpectedly.";
  }

  result = resp.success;
  return testing::AssertionSuccess();
}

/**
 * @brief For testing both the isExecuting method and the is_executing service callback we use parameterized tests.
 */
class PilzJointTrajectoryControllerIsExecutingTest : public PilzJointTrajectoryControllerTest,
                                                     public testing::WithParamInterface<InvokeIsExecuting>
{
protected:
  testing::AssertionResult invokeIsExecuting(bool& result);
};

testing::AssertionResult PilzJointTrajectoryControllerIsExecutingTest::invokeIsExecuting(bool& result)
{
  auto invoke_is_executing {GetParam()};
  return invoke_is_executing(controller_, result);
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testNotInitialized)
{
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
  controller_->state_ = controller_->RUNNING;
  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testNotStarted)
{
  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";
  controller_->state_ = controller_->INITIALIZED;

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testStopTrajExecutionAtStart)
{
  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";
  controller_->state_ = controller_->INITIALIZED;

  startController();
  controller_->state_ = controller_->RUNNING;

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_TRUE(is_executing_result) << "Failed to detect stop trajectory execution at start.";

  ros::Duration stop_duration{STOP_TRAJECTORY_DURATION_SEC + 2*DEFAULT_UPDATE_PERIOD_SEC};
  progressInTime(stop_duration);
  updateController();

  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testNotExecutingAfterUnhold)
{
  ASSERT_TRUE(performFullControllerStartup());
  
  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testActionGoalExecution)
{
  ASSERT_TRUE(performFullControllerStartup());

  GoalType goal {generateSimpleGoal()};
  trajectory_action_client_.sendGoal(goal);

  std::chrono::milliseconds movement_timeout {WAIT_FOR_MOVEMENT_STARTED_MSEC};
  EXPECT_TRUE(updateControllerUntil([this](){ return robot_->isMoving(); }, movement_timeout));

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_TRUE(is_executing_result) << "Failed to detect action goal execution";

  ros::Duration default_update_period {DEFAULT_UPDATE_PERIOD_SEC};
  progressInTime(getGoalDuration(goal) + default_update_period);
  updateController();

  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testTrajCommandExecution)
{
  ASSERT_TRUE(performFullControllerStartup());

  GoalType goal {generateSimpleGoal()};
  trajectory_command_publisher_->publish(goal.trajectory);

  std::chrono::milliseconds movement_timeout {WAIT_FOR_MOVEMENT_STARTED_MSEC};
  EXPECT_TRUE(updateControllerUntil([this](){ return robot_->isMoving(); }, movement_timeout));

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_TRUE(is_executing_result) << "Failed to detect trajectory command execution";

  ros::Duration default_update_period {DEFAULT_UPDATE_PERIOD_SEC};
  progressInTime(getGoalDuration(goal) + default_update_period);
  updateController();

  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testStopTrajExecutionAtHold)
{
  ASSERT_TRUE(performFullControllerStartup());

  GoalType goal {generateSimpleGoal()};
  trajectory_action_client_.sendGoal(goal);

  std::chrono::milliseconds movement_timeout {WAIT_FOR_MOVEMENT_STARTED_MSEC};
  EXPECT_TRUE(updateControllerUntil([this](){ return robot_->isMoving(); }, movement_timeout));

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  // run async such that stop trajectory can be executed in the meantime
  std::future<bool> hold_future = std::async(std::launch::async,
                                             [this, &req, &resp]()
                                             {
                                               return controller_->handleHoldRequest(req, resp);
                                             });

  EXPECT_TRUE(waitForActionResult());
  // the following fails due to https://github.com/ros-controls/ros_controllers/issues/174
  // EXPECT_EQ(trajectory_action_client_.getResult()->error_code, control_msgs::FollowJointTrajectoryResult::INVALID_GOAL);

  updateController();

  bool is_executing_result;
  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_TRUE(is_executing_result) << "Failed to detect stop trajectory execution";

  std::chrono::milliseconds hold_timeout {WAIT_FOR_HOLD_FUTURE_MSEC};
  EXPECT_TRUE(updateControllerUntil([&hold_future](){ return isFutureReady(hold_future); }, hold_timeout));
  EXPECT_TRUE(resp.success);

  EXPECT_TRUE(invokeIsExecuting(is_executing_result));
  EXPECT_FALSE(is_executing_result) << "There should be no execution to detect";
}

INSTANTIATE_TEST_CASE_P(MethodAndServiceCallback, PilzJointTrajectoryControllerIsExecutingTest,
                        testing::Values(InvokeIsExecutingMethod, InvokeIsExecutingServiceCallback));

}  // namespace pilz_joint_trajectory_controller


int main(int argc, char **argv)
{
  ros::init(argc, argv, "unittest_pilz_joint_trajectory_controller");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
