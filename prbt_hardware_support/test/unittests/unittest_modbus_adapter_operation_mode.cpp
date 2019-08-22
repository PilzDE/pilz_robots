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
#include <memory>

#include <prbt_hardware_support/modbus_adapter_operation_mode.h>
#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/OperationModes.h>
#include <prbt_hardware_support/register_container.h>
#include <prbt_hardware_support/modbus_msg_in_builder.h>

namespace prbt_hardware_support
{
static constexpr int DEFAULT_QUEUE_SIZE_MODBUS{1};
static const std::string SERVICE_NAME_OPERATION_MODE = "/prbt/get_operation_mode";

static constexpr unsigned int MODBUS_API_VERSION_REQUIRED{2};

static const ModbusApiSpec TEST_API_SPEC{ {modbus_api_spec::VERSION, 1},
                                          {modbus_api_spec::OPERATION_MODE, 11} };

static constexpr double OPERATION_MODE_CHANGE_WAIT_TIME_S{2.0};
static const std::vector<uint16_t> OPERATION_MODES{1, 2, 3};

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
  ~ModbusAdapterOperationModeTest() override;

  /**
   * @brief Wait for a specific change in operation mode to take effect.
   */
  ::testing::AssertionResult waitForOperationMode(unsigned int op_mode, double timeout = OPERATION_MODE_CHANGE_WAIT_TIME_S);

  /**
   * @brief Wait until operation mode service call return the expected value
   */
  ::testing::AssertionResult waitForServiceCallResult(bool expectation, double timeout = OPERATION_MODE_CHANGE_WAIT_TIME_S);

protected:
  ros::AsyncSpinner spinner_{2};
  ros::NodeHandle nh_;
  std::shared_ptr<ModbusAdapterOperationMode> adapter_operation_mode_;
  ros::Publisher modbus_topic_pub_;
  ros::ServiceClient operation_mode_client_;
};

ModbusAdapterOperationModeTest::ModbusAdapterOperationModeTest()
{
  // Initialize for ROS time if not already initialized
  if(!ros::Time::isValid())
  {
    ros::Time::init();
  }

  adapter_operation_mode_.reset(new ModbusAdapterOperationMode(nh_, TEST_API_SPEC));
  modbus_topic_pub_ = nh_.advertise<ModbusMsgInStamped>(TOPIC_MODBUS_READ, DEFAULT_QUEUE_SIZE_MODBUS);
  operation_mode_client_ = nh_.serviceClient<prbt_hardware_support::GetOperationMode>(SERVICE_NAME_OPERATION_MODE);

  spinner_.start();
}

ModbusAdapterOperationModeTest::~ModbusAdapterOperationModeTest()
{
  // Before the destructors of the class members are called, we have
  // to ensure that all topic and service calls done by the AsyncSpinner
  // threads are finished. Otherwise, we sporadically will see threading
  // exceptions like:
  // "boost::mutex::~mutex(): Assertion `!res' failed".
  spinner_.stop();
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
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(ModbusAdapterOperationModeTest, testAdapterOperationModeDtor)
{
  std::shared_ptr<AdapterOperationMode> adapter_op_mode (new AdapterOperationMode(nh_));
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(ModbusAdapterOperationModeTest, testModbusMsgOperationModeWrapperExceptionDtor)
{
  std::shared_ptr<ModbusMsgOperationModeWrapperException> ex (new ModbusMsgOperationModeWrapperException("Test message"));
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(ModbusAdapterOperationModeTest, testModbusMsgOperationModeWrapperDtor)
{
  ModbusMsgInBuilder builder(TEST_API_SPEC);
  builder.setApiVersion(MODBUS_API_VERSION_REQUIRED).setOperationMode(OperationModes::T1);
  std::shared_ptr<ModbusMsgOperationModeWrapper> wrapper (new ModbusMsgOperationModeWrapper(builder.build(ros::Time::now()), TEST_API_SPEC));
}

/**
 * @tests{Get_OperationMode_mechanism,
 *  Tests that operation mode UNKNOWN is returned if the
 *  operation mode register is missing in the modbus message.
 * }
 *
 * Test Sequence:
 *    1. Send modbus message containing operation mode T1.
 *    2. Send modbus message which does not contain an operation mode.
 *
 * Expected Results:
 *    1. The operation mode service returns T1 as operation mode.
 *    2. The operation mode service returns UNKNWON as operation mode.
 */
TEST_F(ModbusAdapterOperationModeTest, testMissingOperationModeRegister)
{
  ModbusMsgInBuilder builder(TEST_API_SPEC);
  builder.setApiVersion(MODBUS_API_VERSION_REQUIRED);

  ROS_DEBUG("+++  Step 1 +++");
  modbus_topic_pub_.publish(builder.setOperationMode(OperationModes::T1).build(ros::Time::now()));
  ASSERT_TRUE(ros::service::waitForService(SERVICE_NAME_OPERATION_MODE, ros::Duration(3)))
      << "Operation mode service does not appear";
  ASSERT_TRUE(waitForOperationMode(OperationModes::T1));

  ModbusMsgInStampedPtr msg {builder.setOperationMode(OperationModes::T1).build(ros::Time::now())};

  ROS_DEBUG("+++  Step 2 +++");
  // Remove operation mode from modbus message
  ASSERT_GT(TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::OPERATION_MODE), TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::VERSION))
      << "For the test to work correctly, the operation mode register has to be stored in the last register.";
  msg->holding_registers.data.erase(--msg->holding_registers.data.end());
  const uint32_t new_offset = TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::VERSION);
  msg->holding_registers.layout.data_offset = new_offset;
  ModbusMsgInBuilder::setDefaultLayout(&(msg->holding_registers.layout), new_offset, static_cast<uint32_t>(msg->holding_registers.data.size()));

  modbus_topic_pub_.publish(msg);
  ASSERT_TRUE(waitForOperationMode(OperationModes::UNKNOWN));
}

/**
 * @tests{Get_OperationMode_mechanism,
 *  Tests the handling of an incoming modbus message informing
 *  about changing operation mode.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message informing about changing operation mode. Repeat for all possible operation modes.
 *
 * Expected Results:
 *  1. The service call is successful and returns the operation modes published above.
 */
TEST_F(ModbusAdapterOperationModeTest, testOperationModeChange)
{
  ModbusMsgInBuilder builder(TEST_API_SPEC);
  builder.setApiVersion(MODBUS_API_VERSION_REQUIRED);
  for (const auto& mode : OPERATION_MODES)
  {
    modbus_topic_pub_.publish(builder.setOperationMode(mode).build(ros::Time::now()));
    ASSERT_TRUE(ros::service::waitForService(SERVICE_NAME_OPERATION_MODE, ros::Duration(3))) << "Service does not appear";
    ASSERT_TRUE(waitForOperationMode(mode));
  }
}

/**
 * @tests{Get_OperationMode_mechanism,
 *  Tests the handling of an incoming modbus message informing
 *  about a disconnect.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message informing about a disconnect.
 *
 * Expected Results:
 *  1. The service call is not successful
 */
TEST_F(ModbusAdapterOperationModeTest, testDisconnect)
{
  ModbusMsgInBuilder builder(TEST_API_SPEC);
  builder.setApiVersion(MODBUS_API_VERSION_REQUIRED).setOperationMode(OperationModes::T1);

  modbus_topic_pub_.publish(builder.build(ros::Time::now()));

  ASSERT_TRUE(ros::service::waitForService(SERVICE_NAME_OPERATION_MODE, ros::Duration(3))) << "Service does not appear";
  ASSERT_TRUE(waitForOperationMode(OperationModes::T1));


  uint32_t offset{0};
  RegCont holding_register;
  ModbusMsgInStampedPtr msg{ModbusMsgInBuilder::createDefaultModbusMsgIn(offset, holding_register)};
  msg->disconnect.data = true;
  modbus_topic_pub_.publish(msg);

  ASSERT_TRUE(waitForOperationMode(OperationModes::UNKNOWN));
}

/**
 * @tests{Get_OperationMode_mechanism,
 *  Tests the handling of an incoming modbus message with
 *  unexpected operation mode.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message with an unexpected operation mode.
 *
 * Expected Results:
 *  1. The service call is not successful.
 */
TEST_F(ModbusAdapterOperationModeTest, testModbusUnexpectedOperationMode)
{
  ModbusMsgInBuilder builder(TEST_API_SPEC);
  builder.setApiVersion(MODBUS_API_VERSION_REQUIRED).setOperationMode(1234 /* stupid value */);
  modbus_topic_pub_.publish(builder.build(ros::Time::now()));

  // Wait for init
  ASSERT_TRUE(ros::service::waitForService(SERVICE_NAME_OPERATION_MODE, ros::Duration(3))) << "Service does not appear";
  ASSERT_TRUE(waitForOperationMode(OperationModes::UNKNOWN));
}

/**
 * @tests{Get_OperationMode_mechanism,
 *  Tests the handling of an incoming modbus message with incorrect api version.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message with operation mode T1 and correct version.
 *  2. Publish modbus message with operation mode T2 and incorrect version.
 *
 * Expected Results:
 *  1. The version is T1
 *  2. The version is Unknown
 */
TEST_F(ModbusAdapterOperationModeTest, testModbusIncorrectApiVersion)
{
  ModbusMsgInBuilder builder(TEST_API_SPEC);

  // Step 1
  builder.setApiVersion(MODBUS_API_VERSION_REQUIRED).setOperationMode(OperationModes::T1);
  modbus_topic_pub_.publish(builder.build(ros::Time::now()));

  ASSERT_TRUE(ros::service::waitForService(SERVICE_NAME_OPERATION_MODE, ros::Duration(3))) << "Service does not appear";
  ASSERT_TRUE(waitForOperationMode(OperationModes::T1));

  // Step 2
  builder.setApiVersion(0 /* wrong version */).setOperationMode(OperationModes::T2);
  modbus_topic_pub_.publish(builder.build(ros::Time::now()));

  ASSERT_TRUE(waitForOperationMode(OperationModes::UNKNOWN));
}

} // namespace prbt_hardware_support

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_modbus_adapter_operation_mode");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
