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
#include <gmock/gmock.h>

#include <thread>
#include <memory>
#include <chrono>
#include <vector>
#include <functional>
#include <condition_variable>
#include <atomic>
#include <string>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>

#include <modbus/modbus.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/IsBrakeTestRequired.h>
#include <prbt_hardware_support/pilz_modbus_server_mock.h>
#include <prbt_hardware_support/pilz_modbus_read_client.h>
#include <prbt_hardware_support/GetOperationMode.h>
#include <prbt_hardware_support/OperationModes.h>
#include <prbt_hardware_support/ros_test_helper.h>

#include <pilz_testutils/async_test.h>

namespace prbt_hardware_support
{

using ::testing::_;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::InvokeWithoutArgs;

static constexpr uint16_t MODBUS_API_VERSION_VALUE {2};
static const std::string SERVICE_OPERATION_MODE = "/prbt/get_operation_mode";

template<class T>
static void initalizeAndRun(T& obj, const char *ip, unsigned int port)
{
  if ( !obj.init(ip, port) )
  {
    ROS_ERROR("Initialization failed.");
    return;
  }
  ROS_INFO_STREAM("Starting Server on " << ip << ":" << port);

  obj.run();
}

/**
 * @brief OperationModeIntegrationTest checks if the chain
 * ModbusServerMock -> ModbusReadClient -> ModbusAdapterOperationMode functions properly
 *
 * @note the test is derived from testing::AsyncTest which allows the asynchronous processes to re-sync
 */
class OperationModeIntegrationTest : public testing::Test, public testing::AsyncTest
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_{"~"};
};

/**
 * @brief Can be used to check that a expected operation mode is eventually return by the service call
 *
 */
::testing::AssertionResult expectOperationModeServiceCallResult
                                             (ros::ServiceClient& service_client,
                                              OperationModes::_value_type expectation,
                                              uint16_t retries)
{
  prbt_hardware_support::GetOperationMode srv;
  for (int i = 0; i<= retries; i++) {
    auto res = service_client.call(srv);
    if(!res)
    {
      return ::testing::AssertionFailure() << "Could not call service";
    }
    if(srv.response.mode.value == expectation){
      return ::testing::AssertionSuccess() << "It took " << i+1 << " tries for the service call.";
    }
    sleep(1); // This then may take {retries*1}seconds.
  }
  return ::testing::AssertionFailure() << "Did not get expected operation mode:"
                                       << "Actual: " << static_cast<int>(srv.response.mode.value)
                                       << " Expected: " << static_cast<int>(expectation);
}

/**
 * @brief Send data via ModbusServerMock -> ModbusReadClient -> ModbusAdapterOperationMode connection
 * and check that the expected result is returned via the service call.
 *
 * @note Due to the asynchronicity of the test each step of the sequence passed successful
 *       allows the next step to be taken. See testing::AsyncTest for details.
 *
 * Test Sequence:
 *    0. Start Modbus-server in separate thread. Make sure that the nodes are up.
 *    1. Send a brake test required message with the correct API version.
 *    2. Send a brake test required message with the correct API version (change another but irrelevant register entry).
 *    3. Send a brake test not-required message with the correct API version.
 *    4. Terminate ModbusServerMock.
 *
 * Expected Results:
 *    0. -
 *    1. A service call is successfull and returns a positive result.
 *    2. A service call is successfull and returns a positive result.
 *    3. A service call is successfull and returns a negative result.
 *    4. -
 */
TEST_F(OperationModeIntegrationTest, testOperationModeRequestAnnouncement)
{
  EXPECT_GE(std::thread::hardware_concurrency(), 2) << "Hardware does not support enough threads";

  /**********
   * Setup *
   **********/
  std::string ip;
  int port;
  ASSERT_TRUE(nh_priv_.getParam("modbus_server_ip", ip));
  ASSERT_TRUE(nh_priv_.getParam("modbus_server_port", port));

  int modbus_register_size, num_registers_to_read, index_of_first_register_to_read;
  ASSERT_TRUE(nh_priv_.getParam("modbus_register_size", modbus_register_size));
  ASSERT_TRUE(nh_priv_.getParam("num_registers_to_read", num_registers_to_read));
  ASSERT_TRUE(nh_priv_.getParam("index_of_first_register_to_read", index_of_first_register_to_read));

  /**********
   * Step 0 *
   **********/
  prbt_hardware_support::PilzModbusServerMock modbus_server(static_cast<unsigned int>(modbus_register_size));

  std::thread modbus_server_thread( &initalizeAndRun<prbt_hardware_support::PilzModbusServerMock>,
                                    std::ref(modbus_server), ip.c_str(), static_cast<unsigned int>(port) );
	prbt_hardware_support::GetOperationMode srv;

  waitForNode("/pilz_modbus_read_client_node");
  waitForNode("/modbus_adapter_operation_mode_node");

  /**********
   * Step 1 *
   **********/
  modbus_server.setHoldingRegister(std::vector<uint16_t> {MODBUS_API_VERSION_VALUE, 0, 0, 0, 0, 0},
                                   index_of_first_register_to_read);


  ros::ServiceClient operation_mode_client =
    nh_.serviceClient<prbt_hardware_support::GetOperationMode>(SERVICE_OPERATION_MODE);
  ros::service::waitForService(SERVICE_OPERATION_MODE, ros::Duration(10));
  ASSERT_TRUE(operation_mode_client.exists());

	EXPECT_TRUE(expectOperationModeServiceCallResult(operation_mode_client, OperationModes::UNKNOWN, 10));

  /**********
   * Step 2 *
   **********/
  modbus_server.setHoldingRegister(std::vector<uint16_t>{MODBUS_API_VERSION_VALUE, 0, 0, 0, 0, 1},
                                   index_of_first_register_to_read);
	EXPECT_TRUE(expectOperationModeServiceCallResult(operation_mode_client, OperationModes::T1, 10));

  /**********
   * Step 3 *
   **********/
  modbus_server.setHoldingRegister(std::vector<uint16_t>{MODBUS_API_VERSION_VALUE, 0, 0, 0, 0, 2},
                                   index_of_first_register_to_read);
	EXPECT_TRUE(expectOperationModeServiceCallResult(operation_mode_client, OperationModes::T2, 10));

  /**********
   * Step 4 *
   **********/
  ;
  modbus_server.setHoldingRegister(std::vector<uint16_t> {MODBUS_API_VERSION_VALUE, 0, 0, 0, 0, 3},
                                   index_of_first_register_to_read);
	EXPECT_TRUE(expectOperationModeServiceCallResult(operation_mode_client, OperationModes::AUTO, 10));

  /**********
   * Step 5 *
   **********/
  modbus_server.setHoldingRegister(std::vector<uint16_t>{MODBUS_API_VERSION_VALUE, 0, 0, 0, 0, 99},
                                   index_of_first_register_to_read);
	EXPECT_TRUE(expectOperationModeServiceCallResult(operation_mode_client, OperationModes::UNKNOWN, 10));

  /**********
   * Step 4 *
   **********/
  modbus_server.terminate();
  modbus_server_thread.join();
}

} // namespace prbt_hardware_support


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "integrationtest_brake_test_required");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
