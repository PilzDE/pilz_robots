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

#include <std_msgs/Bool.h>

#include <prbt_hardware_support/modbus_adapter_brake_test.h>
#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/modbus_msg_in_utils.h>

#include <pilz_testutils/async_test.h>

namespace prbt_hardware_support
{
static constexpr int DEFAULT_QUEUE_SIZE_MODBUS{1};
static const std::string SERVICE_NAME_IS_BRAKE_TEST_REQUIRED = "/prbt/brake_test_required";

static constexpr unsigned int MODBUS_API_VERSION_REQUIRED{2};
static constexpr unsigned int DEFAULT_RETRIES{10};

static const ModbusApiSpec test_api_spec{ {modbus_api_spec::VERSION, 969},
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
  ~ModbusAdapterBrakeTestTest();

  ModbusMsgInStampedPtr createDefaultBrakeTestModbusMsg(bool brake_test_required,
                                                        unsigned int modbus_api_version = MODBUS_API_VERSION_REQUIRED,
                                                        uint32_t brake_test_required_index = test_api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_REQUEST));
  bool expectBrakeTestRequiredServiceCallResult(ros::ServiceClient& brake_test_required_client,
                                                bool expectation,
                                                uint16_t retries = DEFAULT_RETRIES);

protected:
  ros::NodeHandle nh_;
  std::shared_ptr<ModbusAdapterBrakeTest> adapter_brake_test_;
  ros::Publisher modbus_topic_pub_;
  ros::ServiceClient brake_test_required_client_;//(969, 973, 974);
};

ModbusAdapterBrakeTestTest::ModbusAdapterBrakeTestTest()
{
  adapter_brake_test_.reset(new ModbusAdapterBrakeTest(nh_, test_api_spec));
  modbus_topic_pub_ = nh_.advertise<ModbusMsgInStamped>(TOPIC_MODBUS_READ, DEFAULT_QUEUE_SIZE_MODBUS);
  brake_test_required_client_ = nh_.serviceClient<prbt_hardware_support::IsBrakeTestRequired>(SERVICE_NAME_IS_BRAKE_TEST_REQUIRED);
}

ModbusAdapterBrakeTestTest::~ModbusAdapterBrakeTestTest()
{
}

ModbusMsgInStampedPtr ModbusAdapterBrakeTestTest::createDefaultBrakeTestModbusMsg(bool brake_test_required,
                                                                                    unsigned int modbus_api_version,
                                                                                    uint32_t brake_test_required_index)
{
  uint32_t first_index_to_read{test_api_spec.getRegisterDefinition(modbus_api_spec::VERSION)};
  uint32_t last_index_to_read{brake_test_required_index};
  static int msg_time_counter{1};
  std::vector<uint16_t> tab_reg(last_index_to_read - first_index_to_read + 1);
  tab_reg[0] = modbus_api_version;
  tab_reg[last_index_to_read - first_index_to_read] = brake_test_required;
  ModbusMsgInStampedPtr msg{createDefaultModbusMsgIn(first_index_to_read, tab_reg)};
  msg->header.stamp = ros::Time(msg_time_counter++);
  return msg;
}

bool ModbusAdapterBrakeTestTest::expectBrakeTestRequiredServiceCallResult(ros::ServiceClient& brake_test_required_client,
                                                                            bool expectation,
                                                                            uint16_t retries)
{
  prbt_hardware_support::IsBrakeTestRequired srv;
  for (int i = 0; i<= retries; i++) {
    brake_test_required_client.call(srv);
    if(srv.response.result == expectation){
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
 * Tests the handling of an incoming modbus message informing about a required brake test.
 *
 * Test Sequence:
 *  1. Publish modbus message informing about a required brake test.
 *
 * Expected Results:
 *  1. The service returns true
 */
TEST_F(ModbusAdapterBrakeTestTest, testBrakeTestRequired)
{
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(true));
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       true,
                                                       50));
}

/**
 * Tests the handling of an incoming modbus message informing about a brake test not being required.
 *
 * Test Sequence:
 *  1. Publish modbus message informing about a brake test not being required.
 *
 * Expected Results:
 *  1. The service returns false
 */
TEST_F(ModbusAdapterBrakeTestTest, testBrakeTestNotRequired)
{
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(false));
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       false));
}

/**
 * Tests the handling of an incoming modbus message informing about a disconnect.
 *
 * Test Sequence:
 *  1. Set required state to false
 *  2. Publish modbus message informing about a disconnect.
 *
 * Expected Results:
 *  1. The service returns false
 *  2. The state has not changes (i. e. is still false)
 */
TEST_F(ModbusAdapterBrakeTestTest, testDisconnect)
{
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(false));

  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       false));

  uint32_t offset{0};
  std::vector<uint16_t> holding_register;
  ModbusMsgInStampedPtr msg{createDefaultModbusMsgIn(offset, holding_register)};
  msg->disconnect.data = true;
  modbus_topic_pub_.publish(msg);

  sleep(1);
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       false));
}

/**
 * Tests the handling of an incoming modbus message with incorrect api version.
 *
 * Test Sequence:
 *  1. Set required state to false
 *  2. Publish modbus message with incorrect api version.
 *
 * Expected Results:
 *  1. The service returns false
 *  2. The state has not changes (i. e. is still false)
 */
TEST_F(ModbusAdapterBrakeTestTest, testModbusIncorrectApiVersion)
{
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(false));

  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       false));

  std::vector<uint16_t> holding_register;
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(true, 0));

  sleep(1);
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       false));
}

/**
 * Tests the handling of an incoming modbus message without api version.
 *
 * Test Sequence:
 *  1. Set required state to false
 *  2. Publish modbus message without api version.
 *
 * Expected Results:
 *  1. The service returns false
 *  2. The state has not changes (i. e. is still false)
 */
TEST_F(ModbusAdapterBrakeTestTest, testModbusWithoutApiVersion)
{
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(false));

  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       false));

  auto msg{createDefaultBrakeTestModbusMsg(true, test_api_spec.getRegisterDefinition(modbus_api_spec::VERSION),
  test_api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_REQUEST))};
  msg->holding_registers.data.clear();
  modbus_topic_pub_.publish(msg);

  sleep(1);
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       false));
}

/**
 * Tests the handling of an incoming modbus message without a brake test status.
 *
 * Test Sequence:
 *  1. Set required state to false
 *  2. Publish modbus message without a brake test status.
 *
 * Expected Results:
 *  1. The service returns false
 *  2. The state has not changes (i. e. is still false)
 */
TEST_F(ModbusAdapterBrakeTestTest, testBrakeTestRequiredRegisterMissing)
{
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(false));

  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       false));

  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(true,
                                                            test_api_spec.getRegisterDefinition(modbus_api_spec::VERSION),
                                                            test_api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_REQUEST) - 1));

  sleep(1);
  ASSERT_TRUE(expectBrakeTestRequiredServiceCallResult(brake_test_required_client_,
                                                       false));
}

} // namespace prbt_hardware_support

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_modbus_adapter_brake_test");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner_{2};
  spinner_.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
