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

#include <memory>
#include <string>

#include <ros/ros.h>
#include <modbus/modbus.h>

#include <prbt_hardware_support/pilz_modbus_server_mock.h>
#include <prbt_hardware_support/GetOperationMode.h>
#include <prbt_hardware_support/OperationModes.h>
#include <prbt_hardware_support/ros_test_helper.h>
#include <prbt_hardware_support/modbus_api_spec.h>

namespace prbt_hardware_support
{

static constexpr uint16_t MODBUS_API_VERSION_VALUE {2};
static const std::string SERVICE_OPERATION_MODE = "/prbt/get_operation_mode";

/**
 * @brief OperationModeIntegrationTest checks if the chain
 * ModbusServerMock -> ModbusReadClient -> ModbusAdapterOperationMode functions properly
 */
class OperationModeIntegrationTest : public testing::Test
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_{"~"};
};

/**
 * @brief Can be used to check that a expected operation mode is eventually return by the service call
 *
 */
::testing::AssertionResult expectOperationModeServiceCallResult(ros::ServiceClient& service_client,
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
 * @tests{Get_OperationMode_mechanism,
 *  Test that the expected result is returned via the service call.
 * }
 *
 * @note Due to the asynchronicity of the test each step of the sequence passed successful
 *       allows the next step to be taken. See testing::AsyncTest for details.
 *
 * Data send via:
 *  ModbusServerMock -> ModbusReadClient -> ModbusAdapterOperationMode
 *
 * Test Sequence:
 *    0. Start Modbus-server in separate thread. Make sure that the nodes are up.
 *    1. Send message with 0 (unknown) in operation mode register and with the correct API version.
 *    2. Send message with T1 in operation mode register and with the correct API version.
 *    3. Send message with T2 in operation mode register and with the correct API version.
 *    4. Send message with AUTO (unknown) in operation mode register and with the correct API version.
 *    5. Send message with 99 (unknown) in operation mode register and with the correct API version.
 *    6. Terminate ModbusServerMock.
 *
 * Expected Results:
 *    0. -
 *    1. A service call is successfull and returns unknown operation mode
 *    2. A service call is successfull and returns T1 operation mode
 *    3. A service call is successfull and returns T2 operation mode
 *    4. A service call is successfull and returns AUTO operation mode
 *    5. A service call is successfull and returns unknown operation mode
 *    6. -
 */
TEST_F(OperationModeIntegrationTest, testOperationModeRequestAnnouncement)
{
  /**********
   * Setup *
   **********/
  std::string ip;
  int port;
  ASSERT_TRUE(nh_priv_.getParam("modbus_server_ip", ip));
  ASSERT_TRUE(nh_priv_.getParam("modbus_server_port", port));

  ModbusApiSpec api_spec {nh_};

  unsigned int modbus_register_size {api_spec.getMaxRegisterDefinition() + 1U};

  /**********
   * Step 0 *
   **********/
  prbt_hardware_support::PilzModbusServerMock modbus_server(modbus_register_size);

  std::thread modbus_server_thread( &initalizeAndRun<prbt_hardware_support::PilzModbusServerMock>,
                                    std::ref(modbus_server), ip.c_str(), static_cast<unsigned int>(port) );
  prbt_hardware_support::GetOperationMode srv;

  waitForNode("/pilz_modbus_client_node");
  waitForNode("/modbus_adapter_operation_mode_node");

  /**********
   * Step 1 *
   **********/
  ASSERT_TRUE(api_spec.hasRegisterDefinition(modbus_api_spec::VERSION));
  unsigned int version_register = api_spec.getRegisterDefinition(modbus_api_spec::VERSION);

  modbus_server.setHoldingRegister({{version_register, MODBUS_API_VERSION_VALUE}});


  ros::ServiceClient operation_mode_client =
      nh_.serviceClient<prbt_hardware_support::GetOperationMode>(SERVICE_OPERATION_MODE);
  ros::service::waitForService(SERVICE_OPERATION_MODE, ros::Duration(10));
  ASSERT_TRUE(operation_mode_client.exists());

  EXPECT_TRUE(expectOperationModeServiceCallResult(operation_mode_client, OperationModes::UNKNOWN, 10));

  /**********
   * Step 2 *
   **********/
  ASSERT_TRUE(api_spec.hasRegisterDefinition(modbus_api_spec::OPERATION_MODE));
  unsigned int op_mode_register = api_spec.getRegisterDefinition(modbus_api_spec::OPERATION_MODE);

  modbus_server.setHoldingRegister({{op_mode_register, 1}});
	EXPECT_TRUE(expectOperationModeServiceCallResult(operation_mode_client, OperationModes::T1, 10));

  /**********
   * Step 3 *
   **********/
  modbus_server.setHoldingRegister({{op_mode_register, 2}});
	EXPECT_TRUE(expectOperationModeServiceCallResult(operation_mode_client, OperationModes::T2, 10));

  /**********
   * Step 4 *
   **********/
  modbus_server.setHoldingRegister({{op_mode_register, 3}});
	EXPECT_TRUE(expectOperationModeServiceCallResult(operation_mode_client, OperationModes::AUTO, 10));

  /**********
   * Step 5 *
   **********/
  modbus_server.setHoldingRegister({{op_mode_register, 99}});
	EXPECT_TRUE(expectOperationModeServiceCallResult(operation_mode_client, OperationModes::UNKNOWN, 10));

  /**********
   * Step 6 *
   **********/
  modbus_server.terminate();
  modbus_server_thread.join();
}

} // namespace prbt_hardware_support


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "integrationtest_operation_mode");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
