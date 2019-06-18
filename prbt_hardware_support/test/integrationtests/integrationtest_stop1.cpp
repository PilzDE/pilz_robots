/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#include <modbus/modbus.h>

#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/modbus_api_definitions.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/pilz_modbus_server_mock.h>
#include <prbt_hardware_support/pilz_modbus_read_client.h>
#include <prbt_hardware_support/pilz_manipulator_mock.h>

#include <prbt_hardware_support/ros_test_helper.h>

#include <pilz_testutils/async_test.h>

namespace prbt_hardware_support
{

using ::testing::_;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::InvokeWithoutArgs;

static constexpr uint16_t MODBUS_API_VERSION_VALUE {2};

/**
 * @brief Stop1IntegrationTest checks if the chain
 * ModbusServerMock -> ModbusReadClient -> StoModbusAdapter -> ManipulatorMock functions properly
 *
 * @note the test is derived from testing::AsyncTest which allows the asynchronous processes to re-sync
 */
class Stop1IntegrationTest : public testing::Test, public testing::AsyncTest
{

public:
  Stop1IntegrationTest();
  void SetUp();
  void TearDown();

  // Serves both the controller (/hold + /unhold) and the driver (/halt + /recover) services
  ManipulatorMock manipulator;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_{"~"};
};

void Stop1IntegrationTest::SetUp()
{
  std::string hold_service_name;
  ASSERT_TRUE(nh_priv_.getParam("hold_service_name", hold_service_name));

  std::string unhold_service_name;
  ASSERT_TRUE(nh_priv_.getParam("unhold_service_name", unhold_service_name));

  std::string halt_service_name;
  ASSERT_TRUE(nh_priv_.getParam("halt_service_name", halt_service_name));

  std::string recover_service_name;
  ASSERT_TRUE(nh_priv_.getParam("recover_service_name", recover_service_name));

  manipulator.advertiseServices(nh_,
                                hold_service_name,
                                unhold_service_name,
                                halt_service_name,
                                recover_service_name);
}

void Stop1IntegrationTest::TearDown()
{
}

Stop1IntegrationTest::Stop1IntegrationTest()
{
  EXPECT_GE(std::thread::hardware_concurrency(), 2) << "Hardware does not support enough threads";
}

/**
 * @brief Send data via ModbusServerMock -> ModbusReadClient -> StoModbusAdapter -> ManipulatorMock connection
 * and check that the expected service calls occur in the respective order.
 *
 * @note Due to the asynchronicity of the test each step of the sequence passed successful
 *       allows the next step to be taken. See testing::AsyncTest for details.
 *
 * Test Sequence:
 *    0. Start Modbus-server in seperate thread. Make sure that the nodes are up.
 *       Send a STO clear message with the correct API version.
 *    1. Send a STO active message with the correct API version.
 *    2. Send a STO clear message with the correct API version.
 *    3. Terminate Modbus-server to cause a disconnect.
 *
 * Expected Results:
 *    0. The manipulator mock should receive a call to /unhold after that a call to /recover.
 *       No other calls should happen.
 *    1. The manipulator mock should receive a call to /hold after that a call to /halt.
 *       No other calls should happen.
 *    2. The manipulator mock should receive a call to /unhold after that a call to /recover.
 *       No other calls should happen.
 *    3. The manipulator mock should receive a call to /hold after that a call to /halt.
 *       No other calls should happen.
 */
TEST_F(Stop1IntegrationTest, testServiceCallbacks)
{
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

  waitForNode("/pilz_modbus_read_client_node");
  waitForNode("/sto_modbus_adapter_node");

  // We expect:
  {
    InSequence dummy;

    // Call from STO clear
    EXPECT_CALL(manipulator, unholdCb(_,_)).Times(1);
    EXPECT_CALL(manipulator, holdCb(_,_)).Times(0);
    EXPECT_CALL(manipulator, haltCb(_,_)).Times(0);
    EXPECT_CALL(manipulator, recoverCb(_,_)).Times(1).WillOnce(ACTION_OPEN_BARRIER("recover_callback"));
      // Expected came true -> go on
  }

  // This should trigger the expected reaction
  std::vector<uint16_t> initial_holding_register{modbus_api::v2::MODBUS_STO_CLEAR_VALUE, MODBUS_API_VERSION_VALUE};
  modbus_server.setHoldingRegister(initial_holding_register, index_of_first_register_to_read);

  /**********
   * Step 1 *
   **********/
  BARRIER("recover_callback");

  {
    InSequence dummy;

    // Call from STO active
    EXPECT_CALL(manipulator, unholdCb(_,_)).Times(0);
    EXPECT_CALL(manipulator, recoverCb(_,_)).Times(0);
    EXPECT_CALL(manipulator, holdCb(_,_)).Times(1);
    EXPECT_CALL(manipulator, haltCb(_,_)).Times(1).WillOnce(ACTION_OPEN_BARRIER("halt_callback"));
  }

  std::vector<uint16_t> stop_holding_register{modbus_api::v2::MODBUS_STO_ACTIVE_VALUE, MODBUS_API_VERSION_VALUE};
  modbus_server.setHoldingRegister(stop_holding_register, index_of_first_register_to_read);

  /**********
   * Step 2 *
   **********/
  BARRIER("halt_callback");

  {
    InSequence dummy;

    // Call from STO clear
    EXPECT_CALL(manipulator, holdCb(_,_)).Times(0);
    EXPECT_CALL(manipulator, haltCb(_,_)).Times(0);
    EXPECT_CALL(manipulator, unholdCb(_,_)).Times(1);
    EXPECT_CALL(manipulator, recoverCb(_,_)).Times(1).WillOnce(ACTION_OPEN_BARRIER("recover_callback"));

  }

  std::vector<uint16_t> clear_holding_register{modbus_api::v2::MODBUS_STO_CLEAR_VALUE, MODBUS_API_VERSION_VALUE};
  modbus_server.setHoldingRegister(clear_holding_register, index_of_first_register_to_read);

  /**********
   * Step 3 *
   **********/
  BARRIER("recover_callback");
  {
    InSequence dummy;

    // Call from Disconnect
    EXPECT_CALL(manipulator, holdCb(_,_)).Times(1);
    EXPECT_CALL(manipulator, unholdCb(_,_)).Times(0);
    EXPECT_CALL(manipulator, recoverCb(_,_)).Times(0);
    EXPECT_CALL(manipulator, haltCb(_,_)).Times(1).WillOnce(ACTION_OPEN_BARRIER("halt_callback"));
  }

  modbus_server.terminate();
  modbus_server_thread.join();

  /**********
   * Step 4 *
   **********/
  BARRIER("halt_callback"); // Needed for proper async finish
}

} // namespace prbt_hardware_support


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "integrationtest_stop1");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner_{1};
  spinner_.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
