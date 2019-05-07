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

#include <prbt_hardware_support/modbus_brake_test_announcer.h>
#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/modbus_msg_in_utils.h>

#include <pilz_testutils/async_test.h>

namespace prbt_hardware_support
{
static constexpr int DEFAULT_QUEUE_SIZE_MODBUS{1};
static const std::string TOPIC_BRAKE_TEST_REQUIRED = "/prbt/brake_test_required";
static constexpr int DEFAULT_QUEUE_SIZE_BRAKE_TEST{1};

static constexpr unsigned int MODBUS_API_VERSION_REQUIRED{2};

static constexpr ModbusApiSpec test_api_spec(969, 973);

/**
 * @brief Test fixture for unit-tests of the ModbusBrakeTestAnnouncer.
 *
 * Publish messages on the modbus topic and subscribe to the brake_test topic
 * in order to check if the expectations are met.
 */
class ModbusBrakeTestAnnouncerTest : public testing::Test, public testing::AsyncTest
{
public:
  ModbusBrakeTestAnnouncerTest();
  ~ModbusBrakeTestAnnouncerTest();

  ModbusMsgInStampedPtr createDefaultBrakeTestModbusMsg(bool brake_test_required,
                                                        unsigned int modbus_api_version = MODBUS_API_VERSION_REQUIRED,
                                                        uint32_t brake_test_required_index = test_api_spec.braketest_register_);

  MOCK_METHOD1(brake_test_announcer_callback, void(std_msgs::Bool msg));

protected:
  ros::NodeHandle nh_;
  std::shared_ptr<ModbusBrakeTestAnnouncer> brake_test_announcer_;
  ros::Publisher modbus_topic_pub_;
  ros::Subscriber brake_test_topic_sub_;
};

ModbusBrakeTestAnnouncerTest::ModbusBrakeTestAnnouncerTest()
{
  brake_test_announcer_.reset(new ModbusBrakeTestAnnouncer(nh_, test_api_spec));
  modbus_topic_pub_ = nh_.advertise<ModbusMsgInStamped>(TOPIC_MODBUS_READ, DEFAULT_QUEUE_SIZE_MODBUS);
  brake_test_topic_sub_ = nh_.subscribe<std_msgs::Bool>(TOPIC_BRAKE_TEST_REQUIRED,
                                                        DEFAULT_QUEUE_SIZE_BRAKE_TEST,
                                                        &ModbusBrakeTestAnnouncerTest::brake_test_announcer_callback,
                                                        this);
}

ModbusBrakeTestAnnouncerTest::~ModbusBrakeTestAnnouncerTest()
{
}

ModbusMsgInStampedPtr ModbusBrakeTestAnnouncerTest::createDefaultBrakeTestModbusMsg(bool brake_test_required,
                                                                                    unsigned int modbus_api_version,
                                                                                    uint32_t brake_test_required_index)
{
  uint32_t first_index_to_read{test_api_spec.version_register_};
  uint32_t last_index_to_read{brake_test_required_index};
  static int msg_time_counter{1};
  std::vector<uint16_t> tab_reg(last_index_to_read - first_index_to_read + 1);
  tab_reg[0] = modbus_api_version;
  tab_reg[last_index_to_read - first_index_to_read] = brake_test_required;
  ModbusMsgInStampedPtr msg{createDefaultModbusMsgIn(first_index_to_read, tab_reg)};
  msg->header.stamp = ros::Time(msg_time_counter++);
  return msg;
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
 *  1. A message informing about the required brake test arrives on the brake test topic.
 */
TEST_F(ModbusBrakeTestAnnouncerTest, testBrakeTestRequired)
{
  EXPECT_CALL(*this, brake_test_announcer_callback(InformsAboutRequired()))
      .Times(1)
      .WillOnce(ACTION_OPEN_BARRIER_VOID("brake_test_announcer_callback"));
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(true));

  BARRIER("brake_test_announcer_callback");
}

/**
 * Tests the handling of an incoming modbus message informing about a brake test not being required.
 *
 * Test Sequence:
 *  1. Publish modbus message informing about a brake test not being required.
 *
 * Expected Results:
 *  1. A message informing about the brake test not being required arrives on the brake test topic.
 */
TEST_F(ModbusBrakeTestAnnouncerTest, testBrakeTestNotRequired)
{
  EXPECT_CALL(*this, brake_test_announcer_callback(InformsAboutNotRequired()))
      .Times(1)
      .WillOnce(ACTION_OPEN_BARRIER_VOID("brake_test_announcer_callback"));
  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(false));

  BARRIER("brake_test_announcer_callback");
}

/**
 * Tests the handling of an incoming modbus message informing about a disconnect.
 *
 * Test Sequence:
 *  1. Publish modbus message informing about a disconnect.
 *
 * Expected Results:
 *  1. No message is published on the brake test topic.
 */
TEST_F(ModbusBrakeTestAnnouncerTest, testDisconnect)
{
  EXPECT_CALL(*this, brake_test_announcer_callback(::testing::_))
      .Times(0);

  uint32_t offset{0};
  std::vector<uint16_t> holding_register;
  ModbusMsgInStampedPtr msg{createDefaultModbusMsgIn(offset, holding_register)};
  msg->disconnect.data = true;
  modbus_topic_pub_.publish(msg);

  // Cannot use BARRIER here. Sleep prevents early termination.
  sleep(2.0);
}

/**
 * Tests the handling of an incoming modbus message with incorrect api version.
 *
 * Test Sequence:
 *  1. Publish modbus message with incorrect api version.
 *
 * Expected Results:
 *  1. No message is published on the brake test topic.
 */
TEST_F(ModbusBrakeTestAnnouncerTest, testModbusIncorrectApiVersion)
{
  EXPECT_CALL(*this, brake_test_announcer_callback(::testing::_))
      .Times(0);

  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(true, 0));

  // Cannot use BARRIER here. Sleep prevents early termination.
  sleep(2.0);
}

/**
 * Tests the handling of an incoming modbus message without a brake test status.
 *
 * Test Sequence:
 *  1. Publish modbus message without a brake test status.
 *
 * Expected Results:
 *  1. No message is published on the brake test topic.
 */
TEST_F(ModbusBrakeTestAnnouncerTest, testBrakeTestRequiredRegisterMissing)
{
  EXPECT_CALL(*this, brake_test_announcer_callback(::testing::_))
      .Times(0);

  modbus_topic_pub_.publish(createDefaultBrakeTestModbusMsg(true,
                                                            test_api_spec.version_register_,
                                                            test_api_spec.braketest_register_ - 1));

  // Cannot use BARRIER here. Sleep prevents early termination.
  sleep(2.0);
}

/**
 * Tests the handling of an incoming modbus message without an API version.
 *
 * Test Sequence:
 *  1. Publish modbus message without an API version.
 *
 * Expected Results:
 *  1. No message is published on the brake test topic.
 */
TEST_F(ModbusBrakeTestAnnouncerTest, testModbusApiVersionMissing)
{
  EXPECT_CALL(*this, brake_test_announcer_callback(::testing::_))
      .Times(0);

  auto msg{createDefaultBrakeTestModbusMsg(true, test_api_spec.version_register_, test_api_spec.braketest_register_)};
  msg->holding_registers.data.clear();
  modbus_topic_pub_.publish(msg);

  // Cannot use BARRIER here. Sleep prevents early termination.
  sleep(2.0);
}

} // namespace prbt_hardware_support

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_modbus_brake_test_announcer");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner_{2};
  spinner_.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
