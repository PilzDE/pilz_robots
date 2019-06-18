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
#include <algorithm>

#include <prbt_hardware_support/modbus_adapter_operation_mode.h>
#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/modbus_msg_in_utils.h>
#include <prbt_hardware_support/OperationModes.h>

namespace prbt_hardware_support
{
static constexpr int DEFAULT_QUEUE_SIZE_MODBUS{1};
static const std::string SERVICE_NAME_OPERATION_MODE = "/prbt/get_operation_mode";

static constexpr unsigned int MODBUS_API_VERSION_REQUIRED{2};

static const ModbusApiSpec test_api_spec{ {modbus_api_spec::VERSION, 1},
                                          {modbus_api_spec::OPERATION_MODE, 11} };

static constexpr double OPERATION_MODE_CHANGE_WAIT_TIME_S{2.0};
static const std::vector<unsigned int> OPERATION_MODES{1, 2, 3};

/**
 * @brief Test fixture for unit-tests of the ModbusAdapterOperationMode.
 *
 * Publish messages on the modbus topic and call the operation_mode service
 * in order to check if the expectations are met.
 */
class ModbusAdapterOperationModeTest : public testing::Test
{
public:
  ModbusAdapterOperationModeTest();

  /**
   * @brief Wait for a specific change in operation mode to take effect.
   */
  ::testing::AssertionResult waitForOperationMode(unsigned int op_mode, double timeout = OPERATION_MODE_CHANGE_WAIT_TIME_S);

  /**
   * @brief Wait until operation mode service call return the expected value
   */
  ::testing::AssertionResult waitForServiceCallResult(bool expectation, double timeout = OPERATION_MODE_CHANGE_WAIT_TIME_S);

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

::testing::AssertionResult ModbusAdapterOperationModeTest::waitForServiceCallResult(bool expectation, double timeout)
{
  ros::Rate rate(10.0);
  ros::Time start{ros::Time::now()};
  while (ros::Time::now() - start < ros::Duration(timeout))
  {
    prbt_hardware_support::GetOperationMode srv;
    if (expectation == operation_mode_client_.call(srv))
    {
      return ::testing::AssertionSuccess();
    }
    rate.sleep();
  }

  return ::testing::AssertionFailure() << "Service " << operation_mode_client_.getService() << " did not return the "
                                       << "exptected result";
}

/**
 * Tests the handling of an incoming modbus message informing about changing operation mode.
 *
 * Test Sequence:
 *  1. Publish modbus message informing about changing operation mode. Repeat for all possible operation modes.
 *
 * Expected Results:
 *  1. The service call is successful and returns the operation modes published above.
 */
TEST_F(ModbusAdapterOperationModeTest, testOperationModeChange)
{
  for (auto mode : OPERATION_MODES)
  {
    modbus_topic_pub_.publish(createDefaultOpModeModbusMsg(
      mode,
      MODBUS_API_VERSION_REQUIRED,
      test_api_spec.getRegisterDefinition(modbus_api_spec::OPERATION_MODE),
      test_api_spec.getRegisterDefinition(modbus_api_spec::VERSION))
      );
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
  modbus_topic_pub_.publish(createDefaultOpModeModbusMsg(
    OperationModes::T1,
    MODBUS_API_VERSION_REQUIRED,
    test_api_spec.getRegisterDefinition(modbus_api_spec::OPERATION_MODE),
    test_api_spec.getRegisterDefinition(modbus_api_spec::VERSION))
  );

  ASSERT_TRUE(ros::service::waitForService(SERVICE_NAME_OPERATION_MODE, ros::Duration(3))) << "Service does not appear";
  ASSERT_TRUE(waitForOperationMode(OperationModes::T1));


  uint32_t offset{0};
  std::vector<uint16_t> holding_register;
  ModbusMsgInStampedPtr msg{createDefaultModbusMsgIn(offset, holding_register)};
  msg->disconnect.data = true;
  modbus_topic_pub_.publish(msg);

  ASSERT_TRUE(waitForOperationMode(OperationModes::UNKNOWN));
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
      modbus_topic_pub_.publish(createDefaultOpModeModbusMsg(
      1234, /* stupid value */
      MODBUS_API_VERSION_REQUIRED,
      test_api_spec.getRegisterDefinition(modbus_api_spec::OPERATION_MODE),
      test_api_spec.getRegisterDefinition(modbus_api_spec::VERSION))
  );

  // Wait for init
  ASSERT_TRUE(ros::service::waitForService(SERVICE_NAME_OPERATION_MODE, ros::Duration(3))) << "Service does not appear";
  ASSERT_TRUE(waitForOperationMode(OperationModes::UNKNOWN));
}

/**
 * Tests the handling of an incoming modbus message with incorrect api version.
 *
 * Test Sequence:
 *  1. Publish modbus message with operation mode T1 and correct version.
 *  2. Publish modbus message with operation mode T2 and incorrect version.
 *
 * Expected Results:
 *  1. The version is T1
 *  2. The version is still T1
 */
TEST_F(ModbusAdapterOperationModeTest, testModbusIncorrectApiVersion)
{
  // Step 1
  modbus_topic_pub_.publish(createDefaultOpModeModbusMsg(
    OperationModes::T1,
    MODBUS_API_VERSION_REQUIRED,
    test_api_spec.getRegisterDefinition(modbus_api_spec::OPERATION_MODE),
    test_api_spec.getRegisterDefinition(modbus_api_spec::VERSION))
  );

  ASSERT_TRUE(ros::service::waitForService(SERVICE_NAME_OPERATION_MODE, ros::Duration(3))) << "Service does not appear";
  ASSERT_TRUE(waitForOperationMode(OperationModes::T1));

  // Step 2
  modbus_topic_pub_.publish(
    createDefaultOpModeModbusMsg(
      OperationModes::T2,
      0 /* wrong version */,
      test_api_spec.getRegisterDefinition(modbus_api_spec::OPERATION_MODE),
      test_api_spec.getRegisterDefinition(modbus_api_spec::VERSION))
  );

  ASSERT_TRUE(waitForOperationMode(OperationModes::T1));
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
