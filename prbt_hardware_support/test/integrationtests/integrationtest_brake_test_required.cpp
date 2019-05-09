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
static const std::string TOPIC_BRAKE_TEST_REQUIRED = "/prbt/brake_test_required";
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

// Define matcher for the brake test messages
MATCHER(InformsAboutRequired, "") { return arg.data; }
MATCHER(InformsAboutNotRequired, "") { return !arg.data; }

/**
 * @brief BrakeTestRequiredIntegrationTest checks if the chain
 * ModbusServerMock -> ModbusReadClient -> ModbusBrakeTestAnnouncer functions properly
 *
 * @note the test is derived from testing::AsyncTest which allows the asynchronous processes to re-sync
 */
class BrakeTestRequiredIntegrationTest : public testing::Test, public testing::AsyncTest
{
protected:
  BrakeTestRequiredIntegrationTest();
  /**
   * @brief Start Modbus-server in separate thread. Make sure that the nodes are up.
   */

  MOCK_METHOD1(brake_test_announcer_callback, void(std_msgs::Bool msg));

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_{"~"};
  ros::Subscriber brake_test_topic_sub_;
  ros::ServiceClient is_brake_test_required_client_;
};

BrakeTestRequiredIntegrationTest::BrakeTestRequiredIntegrationTest()
{
  brake_test_topic_sub_ = nh_.subscribe<std_msgs::Bool>(TOPIC_BRAKE_TEST_REQUIRED,
                                                           DEFAULT_QUEUE_SIZE_BRAKE_TEST,
                                                           &BrakeTestRequiredIntegrationTest::brake_test_announcer_callback,
                                                           this);
	is_brake_test_required_client_ = nh_.serviceClient<prbt_hardware_support::IsBrakeTestRequired>("/prbt/is_brake_test_required");
}

/**
 * @brief Send data via ModbusServerMock -> ModbusReadClient -> ModbusBrakeTestAnnouncer connection
 * and check that the expected message arrives on the brake test topic and via the service call.
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
 *    1. A message informing about the required brake test arrives on the brake test topic.
 *       - And the service call gives the same result.
 *    2. No message arrives on the brake test topic.
 *    3. A message informing about the not-required brake test arrives on the brake test topic.
 *       - And the service call gives the same result.
 *    4. No message arrives on the brake test topic.
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
  waitForNode("/modbus_brake_test_announcer_node");

  // We expect:
  {
    InSequence dummy;

    EXPECT_CALL(*this, brake_test_announcer_callback(InformsAboutRequired()))
      .Times(1)
      .WillOnce(ACTION_OPEN_BARRIER_VOID("brake_test_announcer_required"));

    EXPECT_CALL(*this, brake_test_announcer_callback(InformsAboutNotRequired()))
      .Times(1)
      .WillOnce(ACTION_OPEN_BARRIER_VOID("brake_test_announcer_not_required"));
  }

  /**********
   * Step 1 *
   **********/
  std::vector<uint16_t> required_holding_register{MODBUS_API_VERSION_VALUE, 0, 0, 0, 1};
  modbus_server.setHoldingRegister(required_holding_register, index_of_first_register_to_read);

  BARRIER("brake_test_announcer_required");

  /**********
   * Step 2 *
   **********/
  std::vector<uint16_t> required_holding_register_changed{MODBUS_API_VERSION_VALUE, 1, 0, 0, 1};
  modbus_server.setHoldingRegister(required_holding_register_changed, index_of_first_register_to_read);

	is_brake_test_required_client_.call(srv);
	ASSERT_TRUE(srv.response.result == true);

  /**********
   * Step 3 *
   **********/
  std::vector<uint16_t> not_required_holding_register{MODBUS_API_VERSION_VALUE, 0, 0, 0, 0};
  modbus_server.setHoldingRegister(not_required_holding_register, index_of_first_register_to_read);

	is_brake_test_required_client_.call(srv);
	ASSERT_TRUE(srv.response.result == false);

  BARRIER("brake_test_announcer_not_required");

  /**********
   * Step 4 *
   **********/
  modbus_server.terminate();
  modbus_server_thread.join();

  // Cannot use BARRIER here. Sleep prevents early termination.
  sleep(2.0);
}

} // namespace prbt_hardware_support


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "integrationtest_brake_test_required");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner_{1};
  spinner_.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
