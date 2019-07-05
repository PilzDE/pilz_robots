/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <functional>
#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <prbt_hardware_support/brake_test_executor.h>
#include <prbt_hardware_support/BrakeTestErrorCodes.h>
#include <prbt_hardware_support/joint_states_publisher_mock.h>
#include <prbt_hardware_support/pilz_manipulator_mock.h>

namespace brake_test_executor_test
{

using namespace prbt_hardware_support;
using namespace testing;

static const std::string BRAKE_TEST_SERVICE_NAME{"/prbt/execute_braketest"};

static const std::string BRAKETEST_ADAPTER_SERVICE_NAME{"/prbt/braketest_adapter_node/trigger_braketest"};

static const std::string CONTROLLER_HOLD_MODE_SERVICE_NAME{"/prbt/manipulator_joint_trajectory_controller/hold"};
static const std::string CONTROLLER_UNHOLD_MODE_SERVICE_NAME{"/prbt/manipulator_joint_trajectory_controller/unhold"};

class BrakeTestExecutorTest : public Test
{
public:
  BrakeTestExecutorTest();

  MOCK_METHOD2(triggerBrakeTest, bool(BrakeTest::Request &, BrakeTest::Response &));

protected:
  ros::NodeHandle nh_;
  ManipulatorMock manipulator_;
};

BrakeTestExecutorTest::BrakeTestExecutorTest()
{
  manipulator_.advertiseHoldService(nh_, CONTROLLER_HOLD_MODE_SERVICE_NAME);
  manipulator_.advertiseUnholdService(nh_, CONTROLLER_UNHOLD_MODE_SERVICE_NAME);
}

/**
 * @brief Test execution of brake tests while robot is not moving.
 *
 * Test Sequence:
 *  0. Setup Server for triggering the braketest
 *  1. Set expectations and action on service calls
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *
 * Expected Results:
 *  0. Executor is created without problems, a client can attach to its service
 *  1. -
 *  2. -
 *  3. Brake tests are executed successfully. In strict order:
 *     - Hold mode is triggered
 *     - Brake test execution is triggered
 *     - Unhold is triggered
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestTriggeringRobotNotMoving)
{
  /**********
   * Step 0 *
   **********/
  ros::ServiceServer service = nh_.advertiseService<BrakeTestExecutorTest, BrakeTest::Request, BrakeTest::Response>
          (BRAKETEST_ADAPTER_SERVICE_NAME, &BrakeTestExecutorTest::triggerBrakeTest, this);

  BrakeTestExecutor brake_test_executor(this->nh_);

  ros::ServiceClient brake_test_srv_client_ = nh_.serviceClient<BrakeTest>(BRAKE_TEST_SERVICE_NAME);
  ASSERT_TRUE(brake_test_srv_client_.exists()) << "Brake test service not available.";

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(manipulator_, holdCb(_, _)).WillOnce(Return(true));

    EXPECT_CALL(*this, triggerBrakeTest(_, _))
        .Times(1)
        .WillOnce(testing::Invoke(
            [](BrakeTest::Request &, BrakeTest::Response &res) {
              res.success = true;
              return true;
            }));

    EXPECT_CALL(manipulator_, unholdCb(_, _)).WillOnce(Return(true));
  }

  /**********
   * Step 2 *
   **********/
  JointStatesPublisherMock joint_states_pub;
  joint_states_pub.startAsync();

  // /**********
  //  * Step 3 *
  //  **********/
  BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
  EXPECT_TRUE(srv.response.success) << "Brake tests failed unexpectedly. Message: " << srv.response.error_msg;
}

/**
 * @brief Test execution of brake tests while robot is moving.
 *
 * Test Sequence:
 *  0. Setup Server for triggering the braketest
 *  1. Set expectations and action on service calls
 *  2. Publish dynamic(moving) joint states.
 *  3. Call brake test service.
 *
 * Expected Results:
 *  0. Executor is created without problems, a client can attach to its service
 *  1. -
 *  2. -
 *  3. Brake tests cannot be triggered. Respective error message is returned. The hold service is not called.
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestServiceWithRobotMotion)
{
  ros::ServiceServer service = nh_.advertiseService<BrakeTestExecutorTest, BrakeTest::Request, BrakeTest::Response>
          (BRAKETEST_ADAPTER_SERVICE_NAME, &BrakeTestExecutorTest::triggerBrakeTest, this);

  BrakeTestExecutor brake_test_executor(this->nh_);

  ros::ServiceClient brake_test_srv_client_ = nh_.serviceClient<BrakeTest>(BRAKE_TEST_SERVICE_NAME);
  ASSERT_TRUE(brake_test_srv_client_.exists()) << "Brake test service not available.";
  /**********
   * Step 1 *
   **********/
  EXPECT_CALL(*this, triggerBrakeTest(_, _)).Times(0);
  EXPECT_CALL(manipulator_, holdCb(_, _)).Times(0);
  /**********
   * Step 2 *
   **********/
  JointStatesPublisherMock joint_states_pub;
  joint_states_pub.startAsync(true);
  /**********
   * Step 3 *
   **********/
  BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
  EXPECT_FALSE(srv.response.success);
  EXPECT_EQ(BrakeTestErrorCodes::ROBOT_MOTION_DETECTED, srv.response.error_code.value);
}

/**
 * @brief Test behaviour if the triggering of the braketest fails
 *
 * Test Sequence:
 *  0. Setup Server for triggering the braketest, Executor and Client
 *  1. Set expectations and action on the service call. Return false on call.
 *  2. Publish static joint states.
 *  3. Call brake test service.
 *
 * Expected Results:
 *  0. Executor is created without problems, a client can attach to its service
 *  1. -
 *  2. -
 *  3. Error is returned upon service call. In strict order:
 *     - Hold mode is triggered
 *     - Brake test execution is triggered
 *     - Unhold is triggered
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestServiceTriggerFails)
{
  /**********
   * Step 0 *
   **********/
  ros::ServiceServer service = nh_.advertiseService<BrakeTestExecutorTest, BrakeTest::Request, BrakeTest::Response>
          (BRAKETEST_ADAPTER_SERVICE_NAME, &BrakeTestExecutorTest::triggerBrakeTest, this);

  BrakeTestExecutor brake_test_executor(this->nh_);

  ros::ServiceClient brake_test_srv_client_ = nh_.serviceClient<BrakeTest>(BRAKE_TEST_SERVICE_NAME);
  ASSERT_TRUE(brake_test_srv_client_.exists()) << "Brake test service not available.";

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(manipulator_, holdCb(_, _)).WillOnce(Return(true));

    EXPECT_CALL(*this, triggerBrakeTest(_, _))
        .Times(1)
        .WillOnce(Return(false));

    EXPECT_CALL(manipulator_, unholdCb(_, _)).WillOnce(Return(true));
  }

  /**********
   * Step 2 *
   **********/
  JointStatesPublisherMock joint_states_pub;
  joint_states_pub.startAsync();

  /**********
   * Step 3 *
   **********/
  BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
  EXPECT_FALSE(srv.response.success) << "Brake tests succeded unexpectedly.";
  EXPECT_EQ(BrakeTestErrorCodes::TRIGGER_BRAKETEST_SERVICE_FAILURE, srv.response.error_code.value);
}

/**
 * @brief Test execution of brake tests when the hold/unhold service calls fail.
 *
 * This is essentially for line coverage.
 *
 * Test Sequence:
 *  0. Setup Server for triggering the braketest
 *  1. Set expectations and action on service calls
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *
 * Expected Results:
 *  0. Executor is created without problems, a client can attach to its service
 *  1. -
 *  2. -
 *  3. Brake tests are executed successfully. In strict order:
 *     - Hold mode is triggered
 *     - Brake test execution is triggered
 *     - Unhold is triggered
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestTriggeringHoldFailing)
{
  /**********
   * Step 0 *
   **********/
  ros::ServiceServer service = nh_.advertiseService<BrakeTestExecutorTest, BrakeTest::Request, BrakeTest::Response>
          (BRAKETEST_ADAPTER_SERVICE_NAME, &BrakeTestExecutorTest::triggerBrakeTest, this);

  BrakeTestExecutor brake_test_executor(this->nh_);

  ros::ServiceClient brake_test_srv_client_ = nh_.serviceClient<BrakeTest>(BRAKE_TEST_SERVICE_NAME);
  ASSERT_TRUE(brake_test_srv_client_.exists()) << "Brake test service not available.";

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(manipulator_, holdCb(_, _)).WillOnce(Return(false));

    EXPECT_CALL(*this, triggerBrakeTest(_, _))
        .Times(1)
        .WillOnce(testing::Invoke(
            [](BrakeTest::Request &, BrakeTest::Response &res) {
              res.success = true;
              return true;
            }));

    EXPECT_CALL(manipulator_, unholdCb(_, _)).WillOnce(Return(false));
  }

  /**********
   * Step 2 *
   **********/
  JointStatesPublisherMock joint_states_pub;
  joint_states_pub.startAsync();

  /**********
   * Step 3 *
   **********/
  BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
  EXPECT_TRUE(srv.response.success) << "Brake tests failed unexpectedly. Message: " << srv.response.error_msg;
}

} // namespace brake_test_executor_test

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_brake_test_executor");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{2};
  spinner.start();

  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
