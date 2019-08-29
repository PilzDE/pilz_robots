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
#include <functional>
#include <memory>

#include <std_msgs/Bool.h>

#include <prbt_hardware_support/modbus_adapter_brake_test.h>
#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/modbus_msg_in_builder.h>
#include <prbt_hardware_support/register_container.h>
#include <prbt_hardware_support/modbus_msg_brake_test_wrapper.h>
#include <prbt_hardware_support/modbus_msg_brake_test_wrapper_exception.h>

#include <pilz_testutils/async_test.h>

namespace prbt_hardware_support
{
static constexpr int DEFAULT_QUEUE_SIZE_MODBUS{1};
static const std::string SERVICE_NAME_IS_BRAKE_TEST_REQUIRED = "/prbt/brake_test_required";

static constexpr unsigned int MODBUS_API_VERSION_REQUIRED{2};
static constexpr unsigned int DEFAULT_RETRIES{10};

static const ModbusApiSpec TEST_API_SPEC{ {modbus_api_spec::VERSION, 969},
                                          {modbus_api_spec::BRAKETEST_REQUEST,973} };

/**
 * @brief Test fixture for unit-tests of the ModbusAdapterBrakeTest.
 *
 * Publish messages on the modbus topic and call the brake_test_required service
 * in order to check if the expectations are met.
 */
class ModbusAdapterBrakeTestTest : public testing::Test, public testing::AsyncTest
{
public:
  ModbusAdapterBrakeTestTest();
  ~ModbusAdapterBrakeTestTest() override;

public:
  ModbusMsgInStampedPtr createDefaultBrakeTestModbusMsg(uint16_t brake_test_required_value,
                                                        unsigned int modbus_api_version = MODBUS_API_VERSION_REQUIRED,
                                                        uint32_t brake_test_required_index = TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::BRAKETEST_REQUEST));
  bool expectBrakeTestRequiredServiceCallResult(ros::ServiceClient& brake_test_required_client,
                                                IsBrakeTestRequiredResponse::_result_type expectation,
                                                uint16_t retries = DEFAULT_RETRIES);

protected:
  ros::AsyncSpinner spinner_{2};

  ros::NodeHandle nh_;
  std::unique_ptr<ModbusAdapterBrakeTest> adapter_brake_test_ {new ModbusAdapterBrakeTest(nh_, TEST_API_SPEC)};
  ros::Publisher modbus_topic_pub_ {nh_.advertise<ModbusMsgInStamped>(TOPIC_MODBUS_READ, DEFAULT_QUEUE_SIZE_MODBUS)};
  ros::ServiceClient brake_test_required_client_ {nh_.serviceClient<prbt_hardware_support::IsBrakeTestRequired>(SERVICE_NAME_IS_BRAKE_TEST_REQUIRED)};//(969, 973, 974);
};

ModbusAdapterBrakeTestTest::ModbusAdapterBrakeTestTest()
{
  spinner_.start();
}

ModbusAdapterBrakeTestTest::~ModbusAdapterBrakeTestTest()
{
  // Before the destructors of the class members are called, we have
  // to ensure that all topic and service calls done by the AsyncSpinner
  // threads are finished. Otherwise, we sporadically will see threading
  // exceptions like:
  // "boost::mutex::~mutex(): Assertion `!res' failed".
  spinner_.stop();
}

ModbusMsgInStampedPtr ModbusAdapterBrakeTestTest::createDefaultBrakeTestModbusMsg(uint16_t brake_test_required_value,
                                                                                  unsigned int modbus_api_version,
                                                                                  uint32_t brake_test_required_index)
{
  uint32_t first_index_to_read{TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::VERSION)};
  uint32_t last_index_to_read{brake_test_required_index};
  static int msg_time_counter{1};
  RegCont tab_reg(last_index_to_read - first_index_to_read + 1);
  tab_reg[0] = static_cast<uint16_t>(modbus_api_version);
  tab_reg[last_index_to_read - first_index_to_read] = brake_test_required_value;
  ModbusMsgInStampedPtr msg{ModbusMsgInBuilder::createDefaultModbusMsgIn(first_index_to_read, tab_reg)};
  msg->header.stamp = ros::Time(msg_time_counter++);
  return msg;
}

bool ModbusAdapterBrakeTestTest::expectBrakeTestRequiredServiceCallResult(ros::ServiceClient& brake_test_required_client,
                                                                          IsBrakeTestRequiredResponse::_result_type expectation,
                                                                          uint16_t retries)
{
  for (int i = 0; i<= retries; ++i)
  {
    prbt_hardware_support::IsBrakeTestRequired srv;
    brake_test_required_client.call(srv);
    if(srv.response.result == expectation)
    {
      ROS_INFO_STREAM("It took " << i+1 << " tries for the service call.");
      return true;
    }
    sleep(1); // This then may take {retries*1}seconds.
  }
  return false;
}

MATCHER(InformsAboutRequired, "") { return arg.data; }
MATCHER(InformsAboutNotRequired, "") { return !arg.data; }

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(ModbusAdapterBrakeTestTest, testModbusMsgBrakeTestWrapperExceptionDtor)
{
  std::shared_ptr<ModbusMsgBrakeTestWrapperException> msg_wrapper{new ModbusMsgBrakeTestWrapperException("Test msg")};
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(ModbusAdapterBrakeTestTest, testModbusMsgBrakeTestWrapperDtor)
{
  {
    std::shared_ptr<ModbusMsgBrakeTestWrapper> msg_wrapper{
      new ModbusMsgBrakeTestWrapper(createDefaultBrakeTestModbusMsg(REGISTER_VALUE_BRAKETEST_REQUIRED), TEST_API_SPEC)};
  }
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Test increases function coverage by ensuring that all Dtor variants
 *  are called.
 * }
 */
TEST_F(ModbusAdapterBrakeTestTest, testAdapterBrakeTestDtor)
{
  std::shared_ptr<AdapterBrakeTest> adapter{new AdapterBrakeTest(nh_)};
}


/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the value returned if no modbus messages are
 * }
 *
 * Test Sequence:
 *  1. Call service without sending a message via modbus before
 *
 * Expected Results:
 *  1. The service returns true, and the it is unknown if a braketest has to be performed
 */
TEST_F(ModbusAdapterBrakeTestTest, testNoMessageReceived)
{
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       IsBrakeTestRequiredResponse::UNKNOWN));
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message informing about
 *  a required brake test.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message informing about a required brake test.
 *
 * Expected Results:
 *  1. The service returns true
 */
TEST_F(ModbusAdapterBrakeTestTest, testBrakeTestRequired)
{
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(REGISTER_VALUE_BRAKETEST_REQUIRED));
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       IsBrakeTestRequiredResponse::REQUIRED,
                                                       50));
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message informing about
 *  a brake test not being required.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message informing about a brake test not being required.
 *
 * Expected Results:
 *  1. The service returns false
 */
TEST_F(ModbusAdapterBrakeTestTest, testBrakeTestNotRequired)
{
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(REGISTER_VALUE_BRAKETEST_NOT_REQUIRED));
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       IsBrakeTestRequiredResponse::NOT_REQUIRED));
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message informing
 *  about a disconnect.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message informing about a disconnect.
 *
 * Expected Results:
 *  1. The state is unknown
 */
TEST_F(ModbusAdapterBrakeTestTest, testDisconnect)
{

  uint32_t offset{0};
  RegCont holding_register;
  ModbusMsgInStampedPtr msg{ModbusMsgInBuilder::createDefaultModbusMsgIn(offset, holding_register)};
  msg->disconnect.data = true;
  modbus_topic_pub_.publish(msg);

  sleep(1);
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       IsBrakeTestRequiredResponse::UNKNOWN));
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message with
 *  incorrect api version.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message with incorrect api version.
 *
 * Expected Results:
 *  1. The state is unknown
 */
TEST_F(ModbusAdapterBrakeTestTest, testModbusIncorrectApiVersion)
{
  RegCont holding_register;
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(REGISTER_VALUE_BRAKETEST_REQUIRED, 0));

  sleep(1);
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       IsBrakeTestRequiredResponse::UNKNOWN));
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message without api version.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message without api version.
 *
 * Expected Results:
 *  1. The state is unknown
 */
TEST_F(ModbusAdapterBrakeTestTest, testModbusWithoutApiVersion)
{
  auto msg{createDefaultBrakeTestModbusMsg(REGISTER_VALUE_BRAKETEST_REQUIRED,
                                           TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::VERSION),
                                           TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::BRAKETEST_REQUEST))};
  msg->holding_registers.data.clear();
  modbus_topic_pub_.publish(msg);

  sleep(1);
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       IsBrakeTestRequiredResponse::UNKNOWN));
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message without
 *  a brake test status.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message without a brake test status.
 *
 * Expected Results:
 *  1. The state is unknown
 */
TEST_F(ModbusAdapterBrakeTestTest, testBrakeTestRequiredRegisterMissing)
{
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(REGISTER_VALUE_BRAKETEST_REQUIRED,
                                                            TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::VERSION),
                                                            TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::BRAKETEST_REQUEST) - 1));

  sleep(1);
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       IsBrakeTestRequiredResponse::UNKNOWN));
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message with a undefined brake test status register value.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message with a undefine brake test status register value.
 *
 * Expected Results:
 *  1. The state is unknown
 */
TEST_F(ModbusAdapterBrakeTestTest, testBrakeTestRequiredRegisterUndefinedValue)
{
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(555 /* some arbitrary value */));

  sleep(1);
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       IsBrakeTestRequiredResponse::UNKNOWN));
}

} // namespace prbt_hardware_support

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_modbus_adapter_brake_test");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
