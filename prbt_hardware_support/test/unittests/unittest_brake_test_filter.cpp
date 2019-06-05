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

#include <string>
#include <memory>
#include <boost/bind.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <prbt_hardware_support/brake_test_filter.h>
#include <prbt_hardware_support/modbus_msg_brake_test_wrapper.h>
#include <prbt_hardware_support/modbus_msg_in_utils.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>

namespace mf = message_filters;

template class message_filters::BrakeTestFilter<prbt_hardware_support::ModbusMsgInStamped>;

namespace brake_test_filter_test
{
using ::testing::_;
using ::testing::AtLeast;
using ::testing::Return;
using ::testing::Throw;
using namespace prbt_hardware_support;

static const ModbusApiSpec test_api_spec{ {modbus_api_spec::VERSION, 696},
                                          {modbus_api_spec::BRAKETEST_REQUEST,737} };

class CallbackReceiver
{
public:
  MOCK_METHOD1(modbusInMsgCallback, void(ModbusMsgInStampedConstPtr msg));
};

class TestPublisher : public ros::Publisher
{
public:
  TestPublisher(const Publisher &rhs) : ros::Publisher::Publisher(rhs)
  {
  }

  template <typename M>
  void publishAndSpin(const boost::shared_ptr<M> &message) const
  {
    this->publish(message);
    ros::spinOnce();
  }
};

/**
		 * @brief BrakeTestFilterTest tests the BrakeTestFilter class
		 */
class BrakeTestFilterTest : public testing::Test
{
public:
  void SetUp() override;

  void TearDown() override;

  std::shared_ptr<CallbackReceiver> callback_receiver_;
};

void BrakeTestFilterTest::SetUp()
{
  this->callback_receiver_.reset(new CallbackReceiver());
}

void BrakeTestFilterTest::TearDown()
{
  this->callback_receiver_ = nullptr;
}

ModbusMsgInStampedPtr createDefaultBrakeTestModbusMsg(bool brake_test_required)
{
  uint32_t first_index_to_read{test_api_spec.getRegisterDefinition(modbus_api_spec::VERSION)};
  uint32_t last_index_to_read{test_api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_REQUEST)};
  static int msg_time_counter{1};
  std::vector<uint16_t> tab_reg(last_index_to_read - first_index_to_read + 1);
  tab_reg[0] = 1;
  tab_reg[last_index_to_read - first_index_to_read] = brake_test_required;
  ModbusMsgInStampedPtr msg{createDefaultModbusMsgIn(first_index_to_read, tab_reg)};
  msg->header.stamp = ros::Time(msg_time_counter++);
  return msg;
}

/**
		 * @brief Tests that the filter properly only passes message to the callback that have a change in brake test state
		 *        when starting with state true.
		 */
TEST_F(BrakeTestFilterTest, testBrakeTestFilteringThroughSubscriberFromTrue)
{
  std::string test_topic_name = "/test_topic";

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<ModbusMsgInStamped>(test_topic_name, 1);
  TestPublisher test_pub(pub);
  message_filters::Subscriber<ModbusMsgInStamped> modbus_sub(nh, test_topic_name, 1);

  mf::BrakeTestFilter<ModbusMsgInStamped> brake_test_filter(modbus_sub, test_api_spec);

  // Register to modbus callback
  brake_test_filter.registerCallback(
      boost::bind(&CallbackReceiver::modbusInMsgCallback, this->callback_receiver_, _1));

  // Multiple messages will be send, only 3 have changed brake test state
  EXPECT_CALL(*(this->callback_receiver_.get()), modbusInMsgCallback(_)).Times(3);

  ModbusMsgInStampedPtr msg = createDefaultBrakeTestModbusMsg(true);
  test_pub.publishAndSpin(msg); // Should trigger - CALL 1
  msg = createDefaultBrakeTestModbusMsg(true);
  test_pub.publishAndSpin(msg); // Should not trigger as state has not changed

  msg = createDefaultBrakeTestModbusMsg(false);
  test_pub.publishAndSpin(msg); // Should trigger - CALL 2
  msg = createDefaultBrakeTestModbusMsg(false);
  test_pub.publishAndSpin(msg); // Should not trigger as state has not changed
  msg = createDefaultBrakeTestModbusMsg(false);
  test_pub.publishAndSpin(msg); // Should not trigger as state has not changed

  msg = createDefaultBrakeTestModbusMsg(true);
  test_pub.publishAndSpin(msg); // Should trigger - CALL 3
  msg = createDefaultBrakeTestModbusMsg(true);
  test_pub.publishAndSpin(msg); // Should not trigger as state has not changed
}

/**
		 * @brief Tests that the filter properly only passes message to the callback that have a change in brake test state
		 *        when starting with state false.
		 */
TEST_F(BrakeTestFilterTest, testBrakeTestFilteringThroughSubscriberFromFalse)
{
  std::string test_topic_name = "/test_topic";

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<ModbusMsgInStamped>(test_topic_name, 1);
  TestPublisher test_pub(pub);
  message_filters::Subscriber<ModbusMsgInStamped> modbus_sub(nh, test_topic_name, 1);

  mf::BrakeTestFilter<ModbusMsgInStamped> brake_test_filter(modbus_sub, test_api_spec);

  // Register to modbus callback
  brake_test_filter.registerCallback(
      boost::bind(&CallbackReceiver::modbusInMsgCallback, this->callback_receiver_, _1));

  // Multiple messages will be send, only 3 have changed brake test state
  EXPECT_CALL(*(this->callback_receiver_.get()), modbusInMsgCallback(_)).Times(3);

  ModbusMsgInStampedPtr msg = createDefaultBrakeTestModbusMsg(false);
  test_pub.publishAndSpin(msg); // Should trigger - CALL 1
  msg = createDefaultBrakeTestModbusMsg(false);
  test_pub.publishAndSpin(msg); // Should not trigger as state has not changed

  msg = createDefaultBrakeTestModbusMsg(true);
  test_pub.publishAndSpin(msg); // Should trigger - CALL 2
  msg = createDefaultBrakeTestModbusMsg(true);
  test_pub.publishAndSpin(msg); // Should not trigger as state has not changed
  msg = createDefaultBrakeTestModbusMsg(true);
  test_pub.publishAndSpin(msg); // Should not trigger as state has not changed

  msg = createDefaultBrakeTestModbusMsg(false);
  test_pub.publishAndSpin(msg); // Should trigger - CALL 3
  msg = createDefaultBrakeTestModbusMsg(false);
  test_pub.publishAndSpin(msg); // Should not trigger as state has not changed
}

} // namespace brake_test_filter_test

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_brake_test_filter");
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init(); // Needed on message construction
  return RUN_ALL_TESTS();
}
