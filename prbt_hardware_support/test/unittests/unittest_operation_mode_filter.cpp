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

#include <prbt_hardware_support/operation_mode_filter.h>
#include <prbt_hardware_support/modbus_msg_in_utils.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/opertion_modes.h>

static constexpr unsigned int MODBUS_API_VERSION_REQUIRED {2};

namespace mf = message_filters;
using namespace prbt_hardware_support;

template class message_filters::OperationModeFilter<prbt_hardware_support::ModbusMsgInStamped>;

class FilterMock : public message_filters::SimpleFilter<ModbusMsgInStamped>
{
  public:
  /**
   * \brief Call all registered callbacks, passing them the specified message
   */
  void signalMessage(const ModbusMsgInStampedConstPtr& msg)
  {
    message_filters::SimpleFilter<ModbusMsgInStamped>::signalMessage(msg);
  }

};


namespace operation_mode_filter_test
{
using ::testing::_;
using ::testing::AtLeast;
using ::testing::Return;
using ::testing::Throw;
using namespace prbt_hardware_support;

static const ModbusApiSpec test_api_spec{ {modbus_api_spec::VERSION, 696},
                                          {"FakeValue", 511},
                                          {modbus_api_spec::OPERATION_MODE,737} };


ModbusMsgInStampedPtr createDefaultBrakeTestModbusMsg(bool brake_test_required)
{
  uint32_t first_index_to_read{test_api_spec.getRegisterDefinition(modbus_api_spec::VERSION)};
  uint32_t last_index_to_read{test_api_spec.getRegisterDefinition(modbus_api_spec::OPERATION_MODE)};
  static int msg_time_counter{1};
  std::vector<uint16_t> tab_reg(last_index_to_read - first_index_to_read + 1);
  tab_reg[0] = 1;
  tab_reg[last_index_to_read - first_index_to_read] = brake_test_required;
  ModbusMsgInStampedPtr msg{createDefaultModbusMsgIn(first_index_to_read, tab_reg)};
  msg->header.stamp = ros::Time(msg_time_counter++);
  return msg;
}

class OperationModeFilterTest : public testing::Test
{
    public:
        MOCK_METHOD1(modbusInMsgCallback, void(ModbusMsgInStampedConstPtr msg));
};

/**
 * @brief Tests that the filter properly only passes message to the callback that have a operation mode
 *        when starting with state true.
 */
TEST_F(OperationModeFilterTest, passOperationModeMsg)
{
    FilterMock first_filter;

    mf::OperationModeFilter<ModbusMsgInStamped> op_mode_filter(first_filter, test_api_spec);

    op_mode_filter.registerCallback(boost::bind(&OperationModeFilterTest::modbusInMsgCallback, this, _1));

    EXPECT_CALL(*this, modbusInMsgCallback(_)).Times(1);

    // Construct valid message
    ModbusMsgInBuilder builder(test_api_spec);
    builder.setApiVersion(MODBUS_API_VERSION_REQUIRED)
           .setOperationMode(OperationMode::T1);

    first_filter.signalMessage(builder.build());
}

/**
 * @brief Tests that the filter properly only passes message to the callback that have a operation mode
 *        when starting with state true.
 */
TEST_F(OperationModeFilterTest, passOnlyChange)
{
    FilterMock first_filter;

    mf::OperationModeFilter<ModbusMsgInStamped> op_mode_filter(first_filter, test_api_spec);

    op_mode_filter.registerCallback(boost::bind(&OperationModeFilterTest::modbusInMsgCallback, this, _1));

    // Construct valid message
    ModbusMsgInBuilder builder(test_api_spec);
    builder.setApiVersion(1);

    EXPECT_CALL(*this, modbusInMsgCallback(_)).Times(1);
    first_filter.signalMessage(builder.setOperationMode(OperationMode::T1).build());

    EXPECT_CALL(*this, modbusInMsgCallback(_)).Times(0);
    first_filter.signalMessage(builder.setOperationMode(OperationMode::T1).build());

    EXPECT_CALL(*this, modbusInMsgCallback(_)).Times(1);
    first_filter.signalMessage(builder.setOperationMode(OperationMode::T2).build());
}

/**
 * @brief Tests that the filter properly only passes message to the callback that have a operation mode
 *        when starting with state true.
 */
TEST_F(OperationModeFilterTest, messageWithOutOperationMode)
{
    FilterMock first_filter;

    mf::OperationModeFilter<ModbusMsgInStamped> op_mode_filter(first_filter, test_api_spec);

    op_mode_filter.registerCallback(boost::bind(&OperationModeFilterTest::modbusInMsgCallback, this, _1));

    EXPECT_CALL(*this, modbusInMsgCallback(_)).Times(0);

    // Construct valid message
    ModbusMsgInBuilder builder(test_api_spec);
    builder.setApiVersion(MODBUS_API_VERSION_REQUIRED)
           .setRegister(511, 5); // Some register

    first_filter.signalMessage(builder.build());

}

} // namespace operation_mode_filter_test

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_operation_mode_filter");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
