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
#include <prbt_hardware_support/WriteModbusRegister.h>
#include <prbt_hardware_support/canopen_chain_node_mock.h>
#include <prbt_hardware_support/joint_states_publisher_mock.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/pilz_manipulator_mock.h>
#include <prbt_hardware_support/pilz_modbus_server_mock.h>
#include <prbt_hardware_support/ros_test_helper.h>

static const std::string EXECUTE_BRAKE_TEST_SERVICE_NAME{"/prbt/execute_braketest"};
static const std::string MODBUS_WRITE_SERVICE_NAME{"/pilz_modbus_client_node/modbus_write"};
static const std::string CONTROLLER_HOLD_MODE_SERVICE_NAME{"/prbt/manipulator_joint_trajectory_controller/hold"};
static const std::string CONTROLLER_UNHOLD_MODE_SERVICE_NAME{"/prbt/manipulator_joint_trajectory_controller/unhold"};

static constexpr double WAIT_FOR_BRAKE_TEST_SERVICE_TIMEOUT_S{5.0};
static constexpr uint16_t MODBUS_BRAKE_TEST_PREPARE_VALUE {0};
static constexpr uint16_t MODBUS_BRAKE_TEST_EXPECTED_VALUE {1};

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test the BrakeTest service node.
 * }
 *
 * Test Sequence:
 *  0. Preparing modbus mock
 *  1. Initialize CANOpen mock, JointStatesPublisher mock and manipulator mock
 *  2. Wait for BrakeTest service
 *  3. Make a service call
 *
 * Expected Results:
 *  0. -
 *  1. -
 *  2. Service is available
 *  3. Service call is successful
 */
TEST(IntegrationtestExecuteBrakeTest, testBrakeTestService)
{
  using namespace prbt_hardware_support;
  ros::NodeHandle nh;
  ros::Duration(30).sleep();

  /**********
   * Step 0 *
   **********/

  /**********
   * Step 1 *
   **********/
  CANOpenChainNodeMock canopen_mock;

  JointStatesPublisherMock joint_states_pub;
  joint_states_pub.startAsync();

  ManipulatorMock manipulator;
  manipulator.advertiseHoldService(nh, CONTROLLER_HOLD_MODE_SERVICE_NAME);
  manipulator.advertiseUnholdService(nh, CONTROLLER_UNHOLD_MODE_SERVICE_NAME);

  /**********
   * Step 2 *
   **********/
  ros::ServiceClient brake_test_srv_client_ = nh.serviceClient<BrakeTest>(EXECUTE_BRAKE_TEST_SERVICE_NAME);
  EXPECT_TRUE(brake_test_srv_client_.waitForExistence(ros::Duration(WAIT_FOR_BRAKE_TEST_SERVICE_TIMEOUT_S)));

  /**********
   * Step 3 *
   **********/
  std::string ip;
  int port;
  ASSERT_TRUE(nh.getParam("modbus_server_ip", ip));
  ASSERT_TRUE(nh.getParam("modbus_server_port", port));

  ModbusApiSpec api_spec {nh};

  unsigned int modbus_register_size {api_spec.getMaxRegisterDefinition() + 1U};

  prbt_hardware_support::PilzModbusServerMock modbus_server(modbus_register_size);

  std::thread modbus_server_thread( &initalizeAndRun<prbt_hardware_support::PilzModbusServerMock>,
                                    std::ref(modbus_server), ip.c_str(), static_cast<unsigned int>(port) );
  ASSERT_TRUE(api_spec.hasRegisterDefinition(modbus_api_spec::BRAKETEST_PERFORMED));
  unsigned int register_perf = api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_PERFORMED);
  ASSERT_TRUE(api_spec.hasRegisterDefinition(modbus_api_spec::BRAKETEST_RESULT));
  unsigned int register_res = api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_RESULT);

  modbus_server.setHoldingRegister({{register_perf, MODBUS_BRAKE_TEST_PREPARE_VALUE}});
  modbus_server.setHoldingRegister({{register_res, MODBUS_BRAKE_TEST_PREPARE_VALUE}});
  BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client_.call(srv));

  /**********
   * Step 4 *
   **********/
//  RegCont res1 = modbus_server.readHoldingRegister(register_perf, 1);
//  EXPECT_TRUE(res1[0] == MODBUS_BRAKE_TEST_EXPECTED_VALUE);
//  RegCont res2 = modbus_server.readHoldingRegister(register_res, 1);
//  EXPECT_TRUE(res2[0] == MODBUS_BRAKE_TEST_EXPECTED_VALUE);

  /**********
   * Step 6 *
   **********/
//  modbus_server.terminate();
//  modbus_server_thread.join();
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
