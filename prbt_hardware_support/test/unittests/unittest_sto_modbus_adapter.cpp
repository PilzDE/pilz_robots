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

#include <string>
#include <vector>
#include <stdexcept>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/modbus_api_definitions.h>
#include <prbt_hardware_support/param_names.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_msg_sto_wrapper.h>
#include <prbt_hardware_support/modbus_msg_in_utils.h>
#include <prbt_hardware_support/sto_modbus_adapter.h>
#include <prbt_hardware_support/pilz_manipulator_mock.h>
#include <prbt_hardware_support/async_test.h>

namespace prbt_hardware_support
{

static const std::string STO_TOPIC {"/stop1"};
static const std::string STO_ADAPTER_NODE_NAME {"/sto_modbus_adapter_node"};

static constexpr bool STO_CLEAR {false};
static constexpr bool STO_ACTIVE {true};

static constexpr int MODBUS_API_VERSION_FOR_TESTING {1};

using namespace prbt_hardware_support;

using ::testing::_;


#define EXPECT_STOP1 \
  EXPECT_CALL(manipulator_, unholdCb(_,_)).Times(0);\
  EXPECT_CALL(manipulator_, recoverCb(_,_)).Times(0);\
  EXPECT_CALL(manipulator_, holdCb(_,_)).Times(1);\
  EXPECT_CALL(manipulator_, haltCb(_,_)).Times(1).WillOnce(ACTION_OPEN_BARRIER(1));\

#define EXPECT_CLEARANCE \
  EXPECT_CALL(manipulator_, holdCb(_,_)).Times(0); \
  EXPECT_CALL(manipulator_, haltCb(_,_)).Times(0); \
  EXPECT_CALL(manipulator_, unholdCb(_,_)).Times(1); \
  EXPECT_CALL(manipulator_, recoverCb(_,_)).Times(1).WillOnce(ACTION_OPEN_BARRIER(1)); \

/**
 * @brief Test fixture class. Sets up a publisher to the modbus_read topic
 * and a manipulator mock which advertises the needed controller and driver services.
 */
class PilzStoModbusAdapterTest : public ::testing::Test, public ::testing::AsyncTest
{
protected:
  void SetUp();

  ModbusMsgInStampedPtr createDefaultStoModbusMsg(bool sto_clear);

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_ {"~"};
  ros::AsyncSpinner spinner_ {2}; // Keep at 2! Instable if only 1 due the service callbacks in threads

  int index_of_first_register_to_read_;
  int num_registers_to_read_;

  ros::Publisher pub_;
  ManipulatorMock manipulator_;

  std::string HOLD_SERVICE_T {PilzStoModbusAdapterNode::HOLD_SERVICE};
  std::string UNHOLD_SERVICE_T {PilzStoModbusAdapterNode::UNHOLD_SERVICE};
  std::string RECOVER_SERVICE_T {PilzStoModbusAdapterNode::RECOVER_SERVICE};
  std::string HALT_SERVICE_T {PilzStoModbusAdapterNode::HALT_SERVICE};
};

void PilzStoModbusAdapterTest::SetUp()
{
  spinner_.start();

  ASSERT_TRUE(nh_private_.getParam(PARAM_INDEX_OF_FIRST_REGISTER_TO_READ_STR, index_of_first_register_to_read_))
      << "No modbus offset given.";

  ASSERT_TRUE(nh_private_.getParam(PARAM_NUM_REGISTERS_TO_READ_STR, num_registers_to_read_))
      << "No modbus size given.";

  pub_ = nh_.advertise<ModbusMsgInStamped>(TOPIC_MODBUS_READ,1);
  EXPECT_EQ(0, pub_.getNumSubscribers());
}

ModbusMsgInStampedPtr PilzStoModbusAdapterTest::createDefaultStoModbusMsg(bool sto_clear)
{
  static int msg_time_counter {1};
  std::vector<uint16_t> tab_reg(num_registers_to_read_);
  tab_reg[0] = sto_clear;
  tab_reg[1] = MODBUS_API_VERSION_FOR_TESTING;
  ModbusMsgInStampedPtr msg {createDefaultModbusMsgIn(index_of_first_register_to_read_, tab_reg)};
  msg->header.stamp = ros::Time(msg_time_counter++);
  return msg;
}

/**
 * @brief Test that the Setup functions properly
 */
TEST_F(PilzStoModbusAdapterTest, testSetup)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T);

  PilzStoModbusAdapterNode adapter_node(nh_);

  EXPECT_EQ(1, pub_.getNumSubscribers());
}

/**
 * @brief Test an expection is thrown if there is no disable service to disable the drives
 */
TEST_F(PilzStoModbusAdapterTest, testSetupNoDisableService)
{
  manipulator_.advertiseHoldService(nh_, HOLD_SERVICE_T);
  manipulator_.advertiseUnholdService(nh_, UNHOLD_SERVICE_T);
  manipulator_.advertiseRecoverService(nh_, RECOVER_SERVICE_T);

  EXPECT_THROW(PilzStoModbusAdapterNode adapter_node(nh_), PilzStoModbusAdapterNodeException);

  EXPECT_EQ(0, pub_.getNumSubscribers());
}

/**
 * @brief Test that no expection is thrown if there is no service for unholding the controller
 */
TEST_F(PilzStoModbusAdapterTest, testSetupNoUnholdService)
{
  manipulator_.advertiseHoldService(nh_, HOLD_SERVICE_T);
  manipulator_.advertiseHaltService(nh_, HALT_SERVICE_T);
  manipulator_.advertiseRecoverService(nh_, RECOVER_SERVICE_T);

  PilzStoModbusAdapterNode adapter_node(nh_);

  EXPECT_EQ(1, pub_.getNumSubscribers());
}

/**
 * @brief Test that no expection is thrown if there is no service for recovering the driver
 */
TEST_F(PilzStoModbusAdapterTest, testSetupNoRecoverService)
{
  manipulator_.advertiseHoldService(nh_, HOLD_SERVICE_T);
  manipulator_.advertiseUnholdService(nh_, UNHOLD_SERVICE_T);
  manipulator_.advertiseHaltService(nh_, HALT_SERVICE_T);

  PilzStoModbusAdapterNode adapter_node(nh_);

  EXPECT_EQ(1, pub_.getNumSubscribers());
}

/**
 * @brief Test that a expection is thrown if there is no service for holding the controller
 */
TEST_F(PilzStoModbusAdapterTest, testSetupNoHoldService)
{
  manipulator_.advertiseUnholdService(nh_, UNHOLD_SERVICE_T);
  manipulator_.advertiseHaltService(nh_, HALT_SERVICE_T);
  manipulator_.advertiseRecoverService(nh_, RECOVER_SERVICE_T);

  EXPECT_THROW(PilzStoModbusAdapterNode adapter_node(nh_), PilzStoModbusAdapterNodeException);

  EXPECT_EQ(0, pub_.getNumSubscribers());
}

/**
 * @brief Tests that a message giving sto clearance is handled correctly
 */
TEST_F(PilzStoModbusAdapterTest, testClearMsg)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T);

  PilzStoModbusAdapterNode adapter_node(nh_);

  EXPECT_CLEARANCE

  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER_STEP(1);
}

/**
 * @brief Tests that a message giving sto clearance is handled correctly
 */
TEST_F(PilzStoModbusAdapterTest, testRemoveUnholdService)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T);

  PilzStoModbusAdapterNode adapter_node(nh_);

  EXPECT_CALL(manipulator_, recoverCb(_,_)).Times(1).WillOnce(ACTION_OPEN_BARRIER(1));

  manipulator_.shutdownUnholdService();
  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER_STEP(1);
}

/**
 * @brief Tests that a message giving sto clearance is handled correctly
 */
TEST_F(PilzStoModbusAdapterTest, testRemoveRecoverService)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T);

  PilzStoModbusAdapterNode adapter_node(nh_);

  EXPECT_CALL(manipulator_, unholdCb(_,_)).Times(1).WillOnce(ACTION_OPEN_BARRIER(1));

  manipulator_.shutdownRecoverService();
  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER_STEP(1);
}

/**
 * @brief Tests that a message giving sto request is handled correctly
 *
 * Controller must be holded and driver must be disabled
 */
TEST_F(PilzStoModbusAdapterTest, testHoldMsg)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T);

  PilzStoModbusAdapterNode adapter_node(nh_);

  EXPECT_STOP1

  pub_.publish(createDefaultStoModbusMsg(STO_ACTIVE));

  BARRIER_STEP(1);
}

/**
 * @brief Tests that a message indicating a disconnect from modbus stops the robot even if the content
 * would give sto clearance
 */
TEST_F(PilzStoModbusAdapterTest, testDisconnectNoStoMsg)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T);

  PilzStoModbusAdapterNode adapter_node(nh_);

  EXPECT_STOP1

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_CLEAR);
  msg->disconnect.data = true;

  pub_.publish(msg);

  BARRIER_STEP(1);
}

/**
 * @brief Tests that a message indicating a disconnect from modbus stops the robot when the msg itself
 * would also require sto to go active
 */
TEST_F(PilzStoModbusAdapterTest, testDisconnectWithStoMsg)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T);

  PilzStoModbusAdapterNode adapter_node(nh_);

  EXPECT_STOP1

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->disconnect.data = true;

  pub_.publish(msg);

  BARRIER_STEP(1);
}

/**
 * @brief Tests that a message indicating a disconnect from modbus stops with no
 * other data defined in the message
 */
TEST_F(PilzStoModbusAdapterTest, testDisconnectPure)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T);

  PilzStoModbusAdapterNode adapter_node(nh_);

  EXPECT_STOP1

  ModbusMsgInStampedPtr msg (new ModbusMsgInStamped());
  msg->header.stamp = ros::Time::now();
  msg->disconnect.data = true;

  pub_.publish(msg);

  BARRIER_STEP(1);
}

/**
 * @brief Tests that stop happens if no version is defined
 */
TEST_F(PilzStoModbusAdapterTest, testNoVersion)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T);

  PilzStoModbusAdapterNode adapter_node(nh_);

  EXPECT_STOP1

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->holding_registers.data.pop_back();

  pub_.publish(msg);

  BARRIER_STEP(1);
}

/**
 * @brief Tests that a stop happens if the version is wrong
 */
TEST_F(PilzStoModbusAdapterTest, testWrongVersion)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T);

  PilzStoModbusAdapterNode adapter_node(nh_);

  EXPECT_STOP1

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->holding_registers.data[1] = 0;

  pub_.publish(msg);

  BARRIER_STEP(1);
}

/**
 * @brief Test that stop happends if no Sto is defined
 */
TEST_F(PilzStoModbusAdapterTest, testNoSto)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T);

  PilzStoModbusAdapterNode adapter_node(nh_);

  EXPECT_STOP1

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->holding_registers.data.erase(msg->holding_registers.data.begin());
  msg->holding_registers.layout.data_offset = modbus_api::MODBUS_REGISTER_API;

  pub_.publish(msg);

  BARRIER_STEP(1);
}

/**
 * @brief Check construction of the exception (essentially for full function coverage)
 */
TEST_F(PilzStoModbusAdapterTest, ExceptionCTOR)
{
  PilzStoModbusAdapterNodeException* exception = new PilzStoModbusAdapterNodeException("test");

  delete exception;
}

/**
 * @brief Check construction of the exception (essentially for full function coverage)
 */
TEST_F(PilzStoModbusAdapterTest, ModbusMsgExceptionCTOR)
{
  ModbusMsgStoWrapperException* exception = new ModbusMsgStoWrapperException("test");

  delete exception;
}

} // namespace prbt_hardware_support

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_sto_modbus_adapter");
  ros::NodeHandle nh_;

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
