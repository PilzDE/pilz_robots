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

#include <functional>
#include <unistd.h>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>

#include <controller_interface/controller_base.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <pilz_control/pilz_joint_trajectory_controller.h>

typedef hardware_interface::JointCommandInterface HWInterface;
typedef trajectory_interface::QuinticSplineSegment<double> Segment;
typedef pilz_joint_trajectory_controller::PilzJointTrajectoryController<Segment, HWInterface> Controller;
typedef std::shared_ptr<Controller> ControllerPtr;

namespace pilz_joint_trajectory_controller
{

static const std::string CONTROLLER_NAMESPACE{"/controller_ns"};
static const std::string TRAJECTORY_ACTION{"/follow_joint_trajectory"};
static const std::string HOLD_SERVICE{"/hold"};
static const std::string UNHOLD_SERVICE{"/unhold"};
static const std::string IS_EXECUTING_SERVICE{"/is_executing"};
static const std::string TRAJECTORY_COMMAND_TOPIC{"/command"};
static const std::string STOP_TRAJECTORY_DURATION_PARAMETER{"stop_trajectory_duration"};

static constexpr double TIME_COMPARISON_TOLERANCE_SEC{0.000001};
static constexpr double STOP_TRAJECTORY_DURATION{0.2};
static constexpr double TIME_SIMULATION_START_SEC{0.1};
static constexpr double DEFAULT_UPDATE_PERIOD_SEC{0.008};

static constexpr unsigned int SLEEP_TIME_MSEC{10};

static void progressInTime(const ros::Duration& period)
{
  ros::Time current_time{ros::Time::now()};
  ros::Time::setNow(current_time + period);
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

protected:
  ControllerPtr controller_;
  HWInterface* hardware_ {new HWInterface()};
  ros::NodeHandle nh_ {"~"};
  ros::NodeHandle controller_nh_ {CONTROLLER_NAMESPACE};
  ros::AsyncSpinner spinner_ {2};
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
        trajectory_action_client_ {CONTROLLER_NAMESPACE + TRAJECTORY_ACTION, true};
  std::shared_ptr<ros::Publisher> trajectory_command_publisher_;
  ros::Time last_update_time_{TIME_SIMULATION_START_SEC};
};

void PilzJointTrajectoryControllerTest::SetUp()
{
  spinner_.start();

  // register joint handle
  double* pos = new double();
  double* vel = new double();
  double* eff = new double();
  hardware_interface::JointStateHandle jsh {"joint1", pos, vel, eff};
  double* cmd = new double();
  hardware_interface::JointHandle jh {jsh, cmd};
  hardware_->registerHandle(jh);

  // set joints parameter
  controller_nh_.setParam("joints", std::vector<std::string>( {"joint1"} ));

  // Setup controller
  controller_ = std::make_shared<Controller>();

  trajectory_command_publisher_ = std::make_shared<ros::Publisher>(
          nh_.advertise<trajectory_msgs::JointTrajectory>(CONTROLLER_NAMESPACE + TRAJECTORY_COMMAND_TOPIC, 1));

  // Set stop trajectory duration on parameter server (will be read in controller_->init())
  controller_nh_.setParam(STOP_TRAJECTORY_DURATION_PARAMETER, STOP_TRAJECTORY_DURATION);

  // Set up simulated time
  ros::Time start_time{TIME_SIMULATION_START_SEC};
  ros::Time::setNow(start_time);
}

void PilzJointTrajectoryControllerTest::startController()
{
  ros::Time current_time{ros::Time::now()};
  controller_->starting(current_time);
  last_update_time_ = current_time;
}

void PilzJointTrajectoryControllerTest::updateController()
{
  ros::Time current_time{ros::Time::now()};
  controller_->update(current_time, current_time - last_update_time_);
  last_update_time_ = current_time;
}

/**
 * @brief Test the initialization of the PilzJointTrajectoryController.
 *
 * Test Sequence:
 *    1. Construct the controller and call init().
 *
 * Expected Results:
 *    1. The hold/unhold services and the is_executing service exist.
 */
TEST_F(PilzJointTrajectoryControllerTest, testInitializiation)
{
  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";

  ASSERT_TRUE(ros::service::exists(controller_nh_.getNamespace() + HOLD_SERVICE, true));
  ASSERT_TRUE(ros::service::exists(controller_nh_.getNamespace() + UNHOLD_SERVICE, true));
  EXPECT_TRUE(ros::service::exists(controller_nh_.getNamespace() + IS_EXECUTING_SERVICE, true));
}

/**
 * @brief Test the hold/unhold service callbacks of the PilzJointTrajectoryController.
 *
 * Test Sequence:
 *    1. Call the hold callback.
 *    2. Call the unhold callback.
 *
 * Expected Results:
 *    1. Callback returns true.
 *    2. Callback returns true.
 */
TEST_F(PilzJointTrajectoryControllerTest, testForceHoldServiceCallback)
{
  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  ASSERT_TRUE(controller_->handleUnHoldRequest(req, resp));

  ASSERT_TRUE(controller_->handleHoldRequest(req, resp));
}

/**
 * @brief Test the update strategy of the PilzJointTrajectoryController while in HOLD mode.
 *
 * Test Sequence:
 *    1. Initialize and start controller and perform initial update
 *    2. Send goal to controller action server and update with small period
 *
 * Expected Results:
 *    1. Controller is executing (entering HOLD mode at startup)
 *    2. Action result is returned and error_code is INVALID_GOAL
 */
TEST_F(PilzJointTrajectoryControllerTest, testUpdateWhileHolding)
{
  /**********
   * Step 1 *
   **********/
  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";
  controller_->state_ = controller_->INITIALIZED;

  startController();
  controller_->state_ = controller_->RUNNING;

  updateController();
  EXPECT_TRUE(controller_->is_executing()) << "Controller is not executing as expected";

  /**********
   * Step 2 *
   **********/
  EXPECT_EQ(1u, hardware_->getNames().size());
  ros::Duration goal_duration{2.0};

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = hardware_->getNames();
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].time_from_start = goal_duration;
  goal.trajectory.points[0].positions = {0.1};

  trajectory_action_client_.sendGoal(goal);

  ros::Duration period{DEFAULT_UPDATE_PERIOD_SEC};
  progressInTime(period);
  updateController();

  trajectory_action_client_.waitForResult();
  EXPECT_EQ(control_msgs::FollowJointTrajectoryResult::INVALID_GOAL, trajectory_action_client_.getResult()->error_code)
      << "Obtained error code " << trajectory_action_client_.getResult()->error_code << " for goal while in HOLD mode.";
}


/**
 * @brief Test for correct return values and elapsed times in case of subsequent calls to hold/unhold function.
 *
 * Test Sequence:
 *    0. Start controller and perform initial update.
 *    1. Request DEFAULT twice.
 *    2. Request HOLDING.
 *    3. Request HOLDING again.
 *
 * Expected Results:
 *    0. Controller is executing (entering HOLD mode at startup).
 *    1. Each Response contains success==True.
 *    2. Response contains success==True. The elapsed time corresponds to the expected stop trajectory duration.
 *    3. Response contains success==True. The second call does not trigger a stop trajectory.
 */
TEST_F(PilzJointTrajectoryControllerTest, testDoubleRequest)
{
  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";
  controller_->state_ = controller_->INITIALIZED;
  startController();

  controller_->state_ = controller_->RUNNING;

  updateController();
  EXPECT_TRUE(controller_->is_executing()) << "Controller is not executing as expected";

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  ASSERT_TRUE(controller_->handleUnHoldRequest(req, resp));
  EXPECT_TRUE(resp.success);
  ASSERT_TRUE(controller_->handleUnHoldRequest(req, resp));
  EXPECT_TRUE(resp.success);

  ros::Time start_hold_time{ros::Time::now()};
  ASSERT_TRUE(controller_->handleHoldRequest(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_NEAR(ros::Time::now().toSec(), start_hold_time.toSec() + STOP_TRAJECTORY_DURATION, TIME_COMPARISON_TOLERANCE_SEC);

  updateController();

  // Second hold should not sleep
  start_hold_time = ros::Time::now();
  ASSERT_TRUE(controller_->handleHoldRequest(req, resp));
  EXPECT_TRUE(resp.success);
  EXPECT_NEAR(ros::Time::now().toSec(), start_hold_time.toSec(), TIME_COMPARISON_TOLERANCE_SEC);

  // Now we should have stopped
  ros::Duration period{DEFAULT_UPDATE_PERIOD_SEC};
  progressInTime(period);
  updateController();
  EXPECT_FALSE(controller_->is_executing());
}

/**
 * @brief Check that a fake start (simply setting state to RUNNING) doesn't trigger that is_executing() returns true.
 *
 * Increases line coverage.
 */
TEST_F(PilzJointTrajectoryControllerTest, testFakeStart)
{
  controller_->state_ = controller_->RUNNING;
  EXPECT_FALSE(controller_->is_executing()) << "Controller is executing unexpectedly";
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants are called.
 */
TEST_F(PilzJointTrajectoryControllerTest, testD0Destructor)
{
  ControllerPtr controller {new Controller()};
}


///////////////////////////////////////////////
//    Parameterized tests for IsExecuting    //
///////////////////////////////////////////////

typedef std::function<bool(ControllerPtr)> IsExecutingType;

static bool isExecutingMethod(ControllerPtr controller)
{
  return controller->is_executing();
}

static bool isExecutingServiceCallback(ControllerPtr controller)
{
  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  ASSERT_TRUE(controller->handleIsExecutingRequest(req, resp))
      << "Callback of is_executing returned false unexpectedly.";
  return resp.success;
}

/**
 * @brief For testing both the isExecuting method and the is_executing service callback we use parameterized tests.
 */
class PilzJointTrajectoryControllerIsExecutingTest : public PilzJointTrajectoryControllerTest,
                                                     public testing::WithParamInterface<IsExecutingType>
{
protected:
  testing::AssertionResult waitForIsExecutingResult(bool expectation);
};

testing::AssertionResult PilzJointTrajectoryControllerIsExecutingTest::waitForIsExecutingResult(bool expectation)
{
  auto is_executing {GetParam()};
  ros::Duration update_period{DEFAULT_UPDATE_PERIOD_SEC};

  while (ros::ok())
  {
    progressInTime(update_period);
    updateController();

    if (is_executing(controller_) == expectation)
    {
      return testing::AssertionSuccess();
    }

    usleep(SLEEP_TIME_MSEC);
  }

  return testing::AssertionFailure()
      << "Controller did not " << (expectation ? "start" : "stop") << " executing as expected.";
}

/**
 * @brief Test the is_executing method of the PilzJointTrajectoryController.
 *
 * This test is analogue to the test of the is_executing callback.
 *
 * Test Sequence:
 *    1. Do nothing
 *    2. Initialize controller
 *    3. Start and update controller with period > stop_trajectory_duration
 *    4. Unhold and update controller
 *    5. Send goal to controller action server and update periodically
 *    6. Update controller with period > (duration of sent goal)
 *    7. Publish goal on command topic and update periodically
 *    8. Update controller with period > (duration of sent goal)
 *    9. Publish goal on command topic and update periodically
 *    10. Hold and update controller
 *
 * Expected Results:
 *    1. is_executing() returns false
 *    2. is_executing() returns false
 *    3. is_executing() returns false
 *    4. is_executing() returns false
 *    5. is_executing() returns true after a while
 *    6. is_executing() returns false
 *    7. is_executing() returns true after a while
 *    8. is_executing() returns false
 *    9. is_executing() returns true after a while
 *    10. is_executing() returns false
 */
TEST_P(PilzJointTrajectoryControllerIsExecutingTest, testIsExecuting)
{
  /**********
   * Step 1 *
   **********/
  EXPECT_FALSE(controller_->is_executing()) << "Controller is executing unexpectedly";

  /**********
   * Step 2 *
   **********/
  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";
  controller_->state_ = controller_->INITIALIZED;

  EXPECT_FALSE(controller_->is_executing()) << "Controller is executing unexpectedly";

  /**********
   * Step 3 *
   **********/
  startController();
  controller_->state_ = controller_->RUNNING;

  updateController();
  ros::Duration stop_duration{STOP_TRAJECTORY_DURATION + DEFAULT_UPDATE_PERIOD_SEC};
  progressInTime(stop_duration);
  updateController();

  EXPECT_FALSE(controller_->is_executing()) << "Controller is executing unexpectedly";

  /**********
   * Step 4 *
   **********/
  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  EXPECT_TRUE(controller_->handleUnHoldRequest(req, resp));
  EXPECT_TRUE(resp.success);

  ros::Duration default_update_period{DEFAULT_UPDATE_PERIOD_SEC};
  progressInTime(default_update_period);
  updateController();

  EXPECT_FALSE(controller_->is_executing()) << "Controller is executing unexpectedly";

  /**********
   * Step 5 *
   **********/
  EXPECT_EQ(1u, hardware_->getNames().size());
  ros::Duration goal_duration{2.0};

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = hardware_->getNames();
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].time_from_start = goal_duration;
  goal.trajectory.points[0].positions = {0.1};

  trajectory_action_client_.sendGoal(goal);
  EXPECT_TRUE(waitForIsExecutingResult(true));

  /**********
   * Step 6 *
   **********/
  progressInTime(goal_duration + default_update_period);
  updateController();
  EXPECT_FALSE(controller_->is_executing()) << "Controller is executing unexpectedly";

  /**********
   * Step 7 *
   **********/
  trajectory_command_publisher_->publish(goal.trajectory);
  EXPECT_TRUE(waitForIsExecutingResult(true));

  /**********
   * Step 8 *
   **********/
  progressInTime(goal_duration + default_update_period);
  updateController();
  EXPECT_FALSE(controller_->is_executing()) << "Controller is executing unexpectedly";

  /**********
   * Step 9 *
   **********/
  trajectory_command_publisher_->publish(goal.trajectory);
  EXPECT_TRUE(waitForIsExecutingResult(true));

  /**********
   * Step 10 *
   **********/
  EXPECT_TRUE(controller_->handleHoldRequest(req, resp));
  EXPECT_TRUE(resp.success);

  progressInTime(default_update_period);
  updateController();

  EXPECT_FALSE(controller_->is_executing()) << "Controller is executing unexpectedly";
}

INSTANTIATE_TEST_CASE_P(MethodAndServiceCallback, PilzJointTrajectoryControllerIsExecutingTest,
                        testing::Values(isExecutingMethod, isExecutingServiceCallback));

}  // namespace pilz_joint_trajectory_controller


int main(int argc, char **argv)
{
  ros::init(argc, argv, "unittest_pilz_joint_trajectory_controller");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
