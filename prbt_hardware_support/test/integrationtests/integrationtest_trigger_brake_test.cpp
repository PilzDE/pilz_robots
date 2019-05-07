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

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <prbt_hardware_support/BrakeTest.h>
#include <prbt_hardware_support/canopen_chain_node_mock.h>
#include <prbt_hardware_support/joint_states_publisher_mock.h>

static const std::string EXECUTE_BRAKE_TEST_SERVICE_NAME{"/prbt/execute_braketest"};
static constexpr double WAIT_FOR_BRAKE_TEST_SERVICE_TIMEOUT_S{5.0};

/**
 * @brief Test the BrakeTest service node.
 *
 * Test Sequence:
 *  1. Initialize CANOpen mock and JointStatesPublisher mock
 *  2. Wait for BrakeTest service
 *  3. Make a service call
 *
 * Expected Results:
 *  1. -
 *  2. Service is available
 *  3. Service call is successful
 */
TEST(IntegrationtestTriggerBrakeTest, testBrakeTestService)
{
  using namespace prbt_hardware_support;

  /**********
   * Step 1 *
   **********/
  CANOpenChainNodeMock canopen_mock;

  JointStatesPublisherMock joint_states_pub;
  joint_states_pub.startAsync();

  /**********
   * Step 2 *
   **********/
  ros::NodeHandle nh;
  ros::ServiceClient brake_test_srv_client_ = nh.serviceClient<BrakeTest>(EXECUTE_BRAKE_TEST_SERVICE_NAME);
  EXPECT_TRUE(brake_test_srv_client_.waitForExistence(ros::Duration(WAIT_FOR_BRAKE_TEST_SERVICE_TIMEOUT_S)));

  /**********
   * Step 3 *
   **********/
  BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client_.call(srv));
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "integrationtest_trigger_brake_test");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{1};
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
