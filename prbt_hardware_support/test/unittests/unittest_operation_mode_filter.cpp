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
#include <prbt_hardware_support/OperationModes.h>

static constexpr unsigned int MODBUS_API_VERSION_REQUIRED {2};

namespace mf = message_filters;
using namespace prbt_hardware_support;


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

class OperationModeFilterTest : public testing::Test
{
    public:
        MOCK_METHOD1(modbusInMsgCallback, void(ModbusMsgInStampedConstPtr msg));
};

/**
 * @brief Tests that the filter properly passes message to the callback that have a operation mode
 */
TEST_F(OperationModeFilterTest, passOperationModeMsg)
{
    FilterMock first_filter;

    mf::OperationModeFilter op_mode_filter(first_filter, test_api_spec);

    op_mode_filter.registerCallback(boost::bind(&OperationModeFilterTest::modbusInMsgCallback, this, _1));

    EXPECT_CALL(*this, modbusInMsgCallback(_)).Times(1);

    // Construct valid message
    ModbusMsgInBuilder builder(test_api_spec);
    builder.setApiVersion(MODBUS_API_VERSION_REQUIRED)
           .setOperationMode(OperationModes::T1);

    first_filter.signalMessage(builder.build());
}

/**
 * @brief Tests that the filter properly only passes message to the callback that have a change in the operation mode.
 */
TEST_F(OperationModeFilterTest, passOnlyChange)
{
    FilterMock first_filter;

    mf::OperationModeFilter op_mode_filter(first_filter, test_api_spec);

    op_mode_filter.registerCallback(boost::bind(&OperationModeFilterTest::modbusInMsgCallback, this, _1));

    // Construct valid message
    ModbusMsgInBuilder builder(test_api_spec);
    builder.setApiVersion(2);

    EXPECT_CALL(*this, modbusInMsgCallback(_)).Times(1);
    first_filter.signalMessage(builder.setOperationMode(OperationModes::T1).build());

    EXPECT_CALL(*this, modbusInMsgCallback(_)).Times(0);
    first_filter.signalMessage(builder.setOperationMode(OperationModes::T1).build());

    EXPECT_CALL(*this, modbusInMsgCallback(_)).Times(1);
    first_filter.signalMessage(builder.setOperationMode(OperationModes::T2).build());
}

/**
 * @brief Tests that the filter does not pass messages without a operation mode.
 */
TEST_F(OperationModeFilterTest, messageWithOutOperationMode)
{
    FilterMock first_filter;

    mf::OperationModeFilter op_mode_filter(first_filter, test_api_spec);

    op_mode_filter.registerCallback(boost::bind(&OperationModeFilterTest::modbusInMsgCallback, this, _1));

    EXPECT_CALL(*this, modbusInMsgCallback(_)).Times(0);

    // Construct valid message
    ModbusMsgInBuilder builder(test_api_spec);
    builder.setApiVersion(MODBUS_API_VERSION_REQUIRED)
           .setRegister(511, 5); // Some register

    first_filter.signalMessage(builder.build());
}


/**
 * @brief Tests that the filter does not pass messages without a version
 */
TEST_F(OperationModeFilterTest, noVersion)
{
    FilterMock first_filter;

    mf::OperationModeFilter op_mode_filter(first_filter, test_api_spec);

    op_mode_filter.registerCallback(boost::bind(&OperationModeFilterTest::modbusInMsgCallback, this, _1));

    // Construct valid message
    ModbusMsgInBuilder builder(test_api_spec);
    builder.setOperationMode(OperationModes::T1);

    EXPECT_CALL(*this, modbusInMsgCallback(_)).Times(0);
    first_filter.signalMessage(builder.build());
}

} // namespace operation_mode_filter_test

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_operation_mode_filter");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
