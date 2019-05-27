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

#include <prbt_hardware_support/ros_test_helper.h>

#include <pilz_testutils/async_test.h>

namespace prbt_hardware_support
{

using ::testing::_;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::InvokeWithoutArgs;

static constexpr uint16_t MODBUS_API_VERSION_VALUE {2};
static const std::string SERVICE_BRAKETEST_REQUIRED = "/prbt/brake_test_required";
static constexpr int DEFAULT_QUEUE_SIZE_BRAKE_TEST {1};

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
 * @brief BrakeTestRequiredIntegrationTest checks if the chain
 * ModbusServerMock -> ModbusReadClient -> ModbusAdapterBrakeTest functions properly
 *
 * @note the test is derived from testing::AsyncTest which allows the asynchronous processes to re-sync
 */
class BrakeTestRequiredIntegrationTest : public testing::Test, public testing::AsyncTest
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_{"~"};
};

::testing::AssertionResult expectBrakeTestRequiredServiceCallResult
                                             (ros::ServiceClient& brake_test_required_client,
                                              bool expectation,
                                              uint16_t retries)
{
  prbt_hardware_support::IsBrakeTestRequired srv;
  for (int i = 0; i<= retries; i++) {
    auto res = brake_test_required_client.call(srv);
    if(!res)
    {
      return ::testing::AssertionFailure() << "Could not call service";
    }
    if(srv.response.result == expectation){
      return ::testing::AssertionSuccess() << "It took " << i+1 << " tries for the service call.";
    }
    sleep(1); // This then may take {retries*1}seconds.
  }
  return ::testing::AssertionFailure() << "Did not get expected brake test result via service";
}

/**
 * @brief Send data via ModbusServerMock -> ModbusReadClient -> ModbusAdapterBrakeTest connection
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
TEST_F(BrakeTestRequiredIntegrationTest, testBrakeTestAnnouncement)
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
	prbt_hardware_support::IsBrakeTestRequired srv;

  waitForNode("/pilz_modbus_read_client_node");
  waitForNode("/modbus_adapter_brake_test_node");

  /**********
   * Step 1 *
   **********/
  std::vector<uint16_t> required_holding_register{MODBUS_API_VERSION_VALUE, 0, 0, 0, 1};
  modbus_server.setHoldingRegister(required_holding_register, index_of_first_register_to_read);


  ros::ServiceClient	is_brake_test_required_client =
    nh_.serviceClient<prbt_hardware_support::IsBrakeTestRequired>(SERVICE_BRAKETEST_REQUIRED);
  ros::service::waitForService(SERVICE_BRAKETEST_REQUIRED, ros::Duration(10));
  ASSERT_TRUE(is_brake_test_required_client.exists());
  ROS_ERROR("Calling service!");

	EXPECT_TRUE(expectBrakeTestRequiredServiceCallResult(is_brake_test_required_client, true, 10));

  /**********
   * Step 2 *
   **********/
  std::vector<uint16_t> required_holding_register_changed{MODBUS_API_VERSION_VALUE, 1, 0, 0, 1};
  modbus_server.setHoldingRegister(required_holding_register_changed, index_of_first_register_to_read);

	EXPECT_TRUE(expectBrakeTestRequiredServiceCallResult(is_brake_test_required_client, true, 10));

  /**********
   * Step 3 *
   **********/
  std::vector<uint16_t> not_required_holding_register{MODBUS_API_VERSION_VALUE, 0, 0, 0, 0};
  modbus_server.setHoldingRegister(not_required_holding_register, index_of_first_register_to_read);

  EXPECT_TRUE(expectBrakeTestRequiredServiceCallResult(is_brake_test_required_client, false, 10));

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
