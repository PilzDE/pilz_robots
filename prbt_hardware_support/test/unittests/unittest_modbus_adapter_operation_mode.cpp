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
#include <algorithm>
#include <functional>

#include <std_msgs/Bool.h>

#include <prbt_hardware_support/modbus_adapter_operation_mode.h>
#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/modbus_msg_in_utils.h>

#include <pilz_testutils/async_test.h>

namespace prbt_hardware_support
{
static constexpr int DEFAULT_QUEUE_SIZE_MODBUS{1};
static const std::string SERVICE_NAME_OPERATION_MODE = "/prbt/get_operation_mode";

static constexpr unsigned int MODBUS_API_VERSION_REQUIRED{2};

static const ModbusApiSpec test_api_spec(1, 11, 111);

static constexpr double OPERATION_MODE_CHANGE_WAIT_TIME_S{2.0};
static const std::vector<unsigned int> OPERATION_MODES{1, 2, 3};

/**
 * @brief Test fixture for unit-tests of the ModbusAdapterOperationMode.
 *
 * Publish messages on the modbus topic and call the operation_mode service
 * in order to check if the expectations are met.
 */
class ModbusAdapterOperationModeTest : public testing::Test, public testing::AsyncTest
{
public:
  ModbusAdapterOperationModeTest();
  ~ModbusAdapterOperationModeTest();

  /**
   * @brief Create modbus msg given api version and operation mode.
   */
  ModbusMsgInStampedPtr createDefaultOpModeModbusMsg(unsigned int operation_mode,
                                                     unsigned int modbus_api_version = MODBUS_API_VERSION_REQUIRED,
                                                     uint32_t operation_mode_index = test_api_spec.operation_mode_register_,
                                                     uint32_t version_index = test_api_spec.version_register_);

  /**
   * @brief Wait for a specific change in operation mode to take effect.
   */
  ::testing::AssertionResult waitForOperationMode(unsigned int op_mode, double timeout = OPERATION_MODE_CHANGE_WAIT_TIME_S);

  /**
   * @brief Wait until operation mode service call fails.
   */
  bool waitForServiceCallFailure(double timeout = OPERATION_MODE_CHANGE_WAIT_TIME_S);

protected:
  ros::NodeHandle nh_;
  std::shared_ptr<ModbusAdapterOperationMode> adapter_operation_mode_;
  ros::Publisher modbus_topic_pub_;
  ros::ServiceClient operation_mode_client_;
};

ModbusAdapterOperationModeTest::ModbusAdapterOperationModeTest()
{
  adapter_operation_mode_.reset(new ModbusAdapterOperationMode(nh_, test_api_spec));
  modbus_topic_pub_ = nh_.advertise<ModbusMsgInStamped>(TOPIC_MODBUS_READ, DEFAULT_QUEUE_SIZE_MODBUS);
  operation_mode_client_ = nh_.serviceClient<prbt_hardware_support::GetOperationMode>(SERVICE_NAME_OPERATION_MODE);
}

ModbusAdapterOperationModeTest::~ModbusAdapterOperationModeTest()
{
}

ModbusMsgInStampedPtr ModbusAdapterOperationModeTest::createDefaultOpModeModbusMsg(unsigned int operation_mode,
                                                                                   unsigned int modbus_api_version,
                                                                                   uint32_t operation_mode_index,
                                                                                   uint32_t version_index)
{
  uint32_t first_index_to_read{std::min(operation_mode_index, version_index)};
  uint32_t last_index_to_read{std::max(operation_mode_index, version_index)};
  static int msg_time_counter{1};
  std::vector<uint16_t> tab_reg(last_index_to_read - first_index_to_read + 1);
  tab_reg[version_index - first_index_to_read] = modbus_api_version;
  tab_reg[operation_mode_index - first_index_to_read] = operation_mode;
  ModbusMsgInStampedPtr msg{createDefaultModbusMsgIn(first_index_to_read, tab_reg)};
  msg->header.stamp = ros::Time(msg_time_counter++);
  return msg;
}

::testing::AssertionResult ModbusAdapterOperationModeTest::waitForOperationMode(unsigned int op_mode, double timeout)
{
  ros::Rate rate(10.0);
  ros::Time start{ros::Time::now()};
  while (ros::Time::now() - start < ros::Duration(timeout))
  {
    prbt_hardware_support::GetOperationMode srv;
    if (!operation_mode_client_.call(srv))
    {
      return ::testing::AssertionFailure() << "Operation mode service call failed unexpectedly.";
    }
    else if (static_cast<int8_t>(op_mode) == srv.response.mode.value)
    {
      return ::testing::AssertionSuccess();
    }
    rate.sleep();
  }

  return ::testing::AssertionFailure() << "Reached timeout waiting for expected operation mode.";
}

bool ModbusAdapterOperationModeTest::waitForServiceCallFailure(double timeout)
{
  ros::Rate rate(10.0);
  ros::Time start{ros::Time::now()};
  while (ros::Time::now() - start < ros::Duration(timeout))
  {
    prbt_hardware_support::GetOperationMode srv;
    if (!operation_mode_client_.call(srv))
    {
      return true;
    }
    rate.sleep();
  }

  return false;
}

/**
 * Tests the handling of an incoming modbus message informing about changing operation mode.
 *
 * Test Sequence:
 *  1. Publish modbus message informing about changing operation mode. Repeat for all possible operation modes.
 *
 * Expected Results:

  ros::Duration(OPERATION_MODE_CHANGE_WAIT_TIME_S).sleep();

  prbt_hardware_support::GetOperationMode srv;
  EXPECT_FALSE(operation_mode_client_.call(srv));
 *  1. The service call is successful and returns the operation modes published above.
 */
TEST_F(ModbusAdapterOperationModeTest, testOperationModeChange)
{
  for (auto mode : OPERATION_MODES)
  {
    modbus_topic_pub_.publish(createDefaultOpModeModbusMsg(mode));
    ASSERT_TRUE(ros::service::waitForService(SERVICE_NAME_OPERATION_MODE, ros::Duration(3))) << "Service does not appear";
    ASSERT_TRUE(waitForOperationMode(mode));
  }
}

/**
 * Tests the handling of an incoming modbus message informing about a disconnect.
 *
 * Test Sequence:
 *  1. Publish modbus message informing about a disconnect.
 *
 * Expected Results:
 *  1. The service call is not successful
 */
TEST_F(ModbusAdapterOperationModeTest, testDisconnect)
{
  uint32_t offset{0};
  std::vector<uint16_t> holding_register;
  ModbusMsgInStampedPtr msg{createDefaultModbusMsgIn(offset, holding_register)};
  msg->disconnect.data = true;
  modbus_topic_pub_.publish(msg);

  ASSERT_TRUE(waitForServiceCallFailure());
}

/**
 * Tests the handling of an incoming modbus message with unexpected operation mode.
 *
 * Test Sequence:
 *  1. Publish modbus message with an unexpected operation mode.
 *
 * Expected Results:
 *  1. The service call is not successful.
 */
TEST_F(ModbusAdapterOperationModeTest, testModbusUnexpectedOperationMode)
{
  std::vector<uint16_t> holding_register;
  modbus_topic_pub_.publish(createDefaultOpModeModbusMsg(0));

  ASSERT_TRUE(waitForServiceCallFailure());
}

/**
 * Tests the handling of an incoming modbus message with incorrect api version.
 *
 * Test Sequence:
 *  1. Publish modbus message with incorrect api version.
 *
 * Expected Results:
 *  1. The service call is not successful.
 */
TEST_F(ModbusAdapterOperationModeTest, testModbusIncorrectApiVersion)
{
  std::vector<uint16_t> holding_register;
  modbus_topic_pub_.publish(createDefaultOpModeModbusMsg(OPERATION_MODES.at(0), 0));

  ASSERT_TRUE(waitForServiceCallFailure());
}

/**
 * Tests the handling of an incoming modbus message with short register range
 *
 * Test Sequence:
 *  1. Publish modbus message with holding registers not covering the expected range
 *
 * Expected Results:
 *  1. The service call is not successful.
 */
TEST_F(ModbusAdapterOperationModeTest, testModbusWithShortRegisterRange)
{
  auto max_required_index = std::max(test_api_spec.version_register_, test_api_spec.operation_mode_register_);
  auto msg{createDefaultOpModeModbusMsg(OPERATION_MODES.at(0),
                                        MODBUS_API_VERSION_REQUIRED,
                                        max_required_index - 1,
                                        max_required_index - 1)};
  modbus_topic_pub_.publish(msg);

  ASSERT_TRUE(waitForServiceCallFailure());
}

} // namespace prbt_hardware_support

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_modbus_adapter_operation_mode");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner_{2};
  spinner_.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
