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
#include <thread>
#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/modbus_api_definitions.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/param_names.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_msg_sto_wrapper.h>
#include <prbt_hardware_support/modbus_msg_in_builder.h>
#include <prbt_hardware_support/modbus_adapter_sto.h>
#include <prbt_hardware_support/pilz_manipulator_mock.h>
#include <prbt_hardware_support/register_container.h>
#include <prbt_hardware_support/wait_for_topic.h>

#include <pilz_testutils/async_test.h>

namespace prbt_hardware_support
{

static const std::string STO_TOPIC {"/stop1"};
static const std::string STO_ADAPTER_NODE_NAME {"/modbus_adapter_sto_node"};

static constexpr bool STO_CLEAR {true};
static constexpr bool STO_ACTIVE {false};

static const ModbusApiSpec test_api_spec{ {modbus_api_spec::VERSION, 513},
                                          {modbus_api_spec::STO, 512} };

static constexpr int MODBUS_API_VERSION_FOR_TESTING {2};

using namespace prbt_hardware_support;

using ::testing::_;
using ::testing::InSequence;
using ::testing::InvokeWithoutArgs;
using ::testing::Return;


#define EXPECT_STOP1 \
  do { \
  EXPECT_CALL(manipulator_, unholdCb(_,_)).Times(0);\
  EXPECT_CALL(manipulator_, recoverCb(_,_)).Times(0);\
  EXPECT_CALL(manipulator_, holdCb(_,_)).Times(1);\
  EXPECT_CALL(manipulator_, isExecutingCb(_,_)).Times(1);\
  EXPECT_CALL(manipulator_, haltCb(_,_)).Times(1).WillOnce(ACTION_OPEN_BARRIER("halt_callback")); }\
  while(false)

#define EXPECT_CLEARANCE \
  do { \
  EXPECT_CALL(manipulator_, holdCb(_,_)).Times(0); \
  EXPECT_CALL(manipulator_, haltCb(_,_)).Times(0); \
  EXPECT_CALL(manipulator_, recoverCb(_,_)).Times(1).WillOnce(Return(true)); \
  EXPECT_CALL(manipulator_, unholdCb(_,_)).Times(1).WillOnce(ACTION_OPEN_BARRIER("unhold_callback")); }\
  while(false)

/**
 * @brief Test fixture class. Sets up a publisher to the modbus_read topic
 * and a manipulator mock which advertises the needed controller and driver services.
 */
class ModbusAdapterStoTest : public ::testing::Test, public ::testing::AsyncTest
{
protected:
  ModbusAdapterStoTest();
  virtual ~ModbusAdapterStoTest() override;

  virtual void SetUp() override;

  ModbusMsgInStampedPtr createDefaultStoModbusMsg(bool sto_clear);
  std::thread asyncConstructor();

protected:
  ros::NodeHandle nh_ {};
  ros::NodeHandle nh_private_ {"~"};
  ros::AsyncSpinner spinner_ {2}; // Keep at 2! Instable if only 1 due the service callbacks in threads

  int index_of_first_register_to_read_ {0};
  int num_registers_to_read_ {0};

  std::unique_ptr<ModbusAdapterSto> modbus_sto_adapter_ {nullptr};
  ManipulatorMock manipulator_;
  ros::Publisher pub_ {nh_.advertise<ModbusMsgInStamped>(TOPIC_MODBUS_READ,1)};

  std::string HOLD_SERVICE_T {ModbusAdapterSto::HOLD_SERVICE};
  std::string UNHOLD_SERVICE_T {ModbusAdapterSto::UNHOLD_SERVICE};
  std::string RECOVER_SERVICE_T {ModbusAdapterSto::RECOVER_SERVICE};
  std::string HALT_SERVICE_T {ModbusAdapterSto::HALT_SERVICE};
  std::string IS_EXECUTING_SERVICE_T {ModbusAdapterSto::IS_EXECUTING_SERVICE};
};

ModbusAdapterStoTest::ModbusAdapterStoTest()
{
  spinner_.start();
}

ModbusAdapterStoTest::~ModbusAdapterStoTest()
{
  // Before the destructors of the class members are called, we have
  // to ensure that all topic and service calls done by the AsyncSpinner
  // threads are finished. Otherwise, we sporadically will see threading
  // exceptions like:
  // "boost::mutex::~mutex(): Assertion `!res' failed".
  spinner_.stop();
}

void ModbusAdapterStoTest::SetUp()
{
  ASSERT_TRUE(nh_private_.getParam(PARAM_INDEX_OF_FIRST_REGISTER_TO_READ_STR, index_of_first_register_to_read_))
      << "No modbus offset given.";

  ASSERT_TRUE(nh_private_.getParam(PARAM_NUM_REGISTERS_TO_READ_STR, num_registers_to_read_))
      << "No modbus size given.";

  EXPECT_EQ(0, pub_.getNumSubscribers());
}

ModbusMsgInStampedPtr ModbusAdapterStoTest::createDefaultStoModbusMsg(bool sto)
{
  static int msg_time_counter {1};
  RegCont tab_reg(static_cast<uint16_t>(num_registers_to_read_));
  tab_reg[0] = sto;
  tab_reg[1] = MODBUS_API_VERSION_FOR_TESTING;
  ModbusMsgInStampedPtr msg {ModbusMsgInBuilder::createDefaultModbusMsgIn(static_cast<uint16_t>(index_of_first_register_to_read_), tab_reg)};
  msg->header.stamp = ros::Time(msg_time_counter++);
  return msg;
}

std::thread ModbusAdapterStoTest::asyncConstructor()
{
  std::thread t([this](){
    ModbusAdapterSto adapter_node(nh_, test_api_spec);
  });
  return t;
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(ModbusAdapterStoTest, testModbusMsgWrapperExceptionDtor)
{
  std::shared_ptr<ModbusMsgWrapperException> es{new ModbusMsgWrapperException("Test msg")};
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(ModbusAdapterStoTest, testModbusMsgStoWrapperDtor)
{
  ModbusMsgInStampedConstPtr msg_const_ptr {createDefaultStoModbusMsg(STO_CLEAR)};
  std::shared_ptr<ModbusMsgStoWrapper> ex {new ModbusMsgStoWrapper(msg_const_ptr, test_api_spec)};
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(ModbusAdapterStoTest, testAdapterStoDtor)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);
  std::shared_ptr<AdapterSto> adapter {new AdapterSto()};
}

/**
 * @brief Test that the Setup functions properly
 */
TEST_F(ModbusAdapterStoTest, testSetup)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  EXPECT_EQ(1, pub_.getNumSubscribers());
}

/**
 * @tests{No_Startup_if_driver_halt_missing,
 *  Test constructor with delayed halt service.
 * }
 *
 * Expected: Constructor blocks until the halt service is available.
 */
TEST_F(ModbusAdapterStoTest, testSetupNoDisableService)
{
  manipulator_.advertiseHoldService(nh_, HOLD_SERVICE_T);
  manipulator_.advertiseUnholdService(nh_, UNHOLD_SERVICE_T);
  manipulator_.advertiseIsExecutingService(nh_, IS_EXECUTING_SERVICE_T);
  manipulator_.advertiseRecoverService(nh_, RECOVER_SERVICE_T);

  auto adapter_thread = asyncConstructor();
  ros::Duration(1.0).sleep();

  EXPECT_EQ(0, pub_.getNumSubscribers()); // the constructor should wait, no subscription yet
  manipulator_.advertiseHaltService(nh_, HALT_SERVICE_T);
  adapter_thread.join();
}

/**
 * @tests{No_Startup_if_controller_unhold_missing,
 *  Test constructor with delayed unhold service.
 * }
 *
 * Expected: Constructor blocks until the unhold service is available.
 */
TEST_F(ModbusAdapterStoTest, testSetupNoUnholdService)
{
  manipulator_.advertiseHoldService(nh_, HOLD_SERVICE_T);
  manipulator_.advertiseIsExecutingService(nh_, IS_EXECUTING_SERVICE_T);
  manipulator_.advertiseHaltService(nh_, HALT_SERVICE_T);
  manipulator_.advertiseRecoverService(nh_, RECOVER_SERVICE_T);

  auto adapter_thread = asyncConstructor();
  ros::Duration(1.0).sleep();

  EXPECT_EQ(0, pub_.getNumSubscribers()); // the constructor should wait, no subscription yet
  manipulator_.advertiseUnholdService(nh_, UNHOLD_SERVICE_T);
  adapter_thread.join();
}

/**
 * @tests{No_Startup_if_driver_recover_missing,
 *  Test constructor with delayed recover service.
 * }
 *
 * Expected: Constructor blocks until the recover service is available.
 */
TEST_F(ModbusAdapterStoTest, testSetupNoRecoverService)
{
  manipulator_.advertiseHoldService(nh_, HOLD_SERVICE_T);
  manipulator_.advertiseUnholdService(nh_, UNHOLD_SERVICE_T);
  manipulator_.advertiseIsExecutingService(nh_, IS_EXECUTING_SERVICE_T);
  manipulator_.advertiseHaltService(nh_, HALT_SERVICE_T);

  auto adapter_thread = asyncConstructor();
  ros::Duration(1.0).sleep();

  EXPECT_EQ(0, pub_.getNumSubscribers()); // the constructor should wait, no subscription yet
  manipulator_.advertiseRecoverService(nh_, RECOVER_SERVICE_T);
  adapter_thread.join();
}

/**
 * @tests{No_Startup_if_controller_hold_missing,
 *  Test that constructor blocks until hold service is available.
 * }
 *
 * Expected: Constructor blocks until the hold service is available.
 */
TEST_F(ModbusAdapterStoTest, testSetupNoHoldService)
{
  manipulator_.advertiseUnholdService(nh_, UNHOLD_SERVICE_T);
  manipulator_.advertiseIsExecutingService(nh_, IS_EXECUTING_SERVICE_T);
  manipulator_.advertiseHaltService(nh_, HALT_SERVICE_T);
  manipulator_.advertiseRecoverService(nh_, RECOVER_SERVICE_T);

  auto adapter_thread = asyncConstructor();
  ros::Duration(1.0).sleep();

  EXPECT_EQ(0, pub_.getNumSubscribers()); // the constructor should wait, no subscription yet
  manipulator_.advertiseHoldService(nh_, HOLD_SERVICE_T);
  adapter_thread.join();
}

/**
 * @brief Tests that a message giving sto clearance is handled correctly
 *
 * @tests{Recover_driver_after_STO_false,
 *  Tests that driver is recovered in case of STO switch: false->true.
 * }
 *
 * @tests{Leave_hold_after_recover,
 *  Tests that new commands are accepted by controller in case of STO switch:
 *  false->true
 * }
 *
 */
TEST_F(ModbusAdapterStoTest, testClearMsg)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  EXPECT_CLEARANCE;

  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("unhold_callback");
}

/**
 * @tests{Controller_service_unhold_optional,
 *  Test system can deal correctly with missing unhold service of controller.
 * }
 */
TEST_F(ModbusAdapterStoTest, testRemoveUnholdService)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  EXPECT_CALL(manipulator_, recoverCb(_,_)).Times(1).WillOnce(ACTION_OPEN_BARRIER("recover_callback"));

  manipulator_.shutdownUnholdService();
  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("recover_callback");
  ros::Duration(1.0).sleep();

  EXPECT_STOP1;

  pub_.publish(createDefaultStoModbusMsg(STO_ACTIVE));

  BARRIER("halt_callback");
}

/**
 * @tests{Controller_service_hold_optional,
 *  Test system can deal correctly with missing hold service of controller.
 * }
 */
TEST_F(ModbusAdapterStoTest, testRemoveHoldService)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  EXPECT_CLEARANCE;

  manipulator_.shutdownHoldService();
  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("unhold_callback");

  EXPECT_CALL(manipulator_, isExecutingCb(_,_)).Times(1);
  EXPECT_CALL(manipulator_, haltCb(_,_)).WillOnce(ACTION_OPEN_BARRIER("halt_callback"));

  pub_.publish(createDefaultStoModbusMsg(STO_ACTIVE));

  BARRIER("halt_callback");

  EXPECT_CLEARANCE;

  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("unhold_callback");
}

/**
 * @tests{Controller_service_is_executing_optional,
 *  Test system can deal correctly with missing is_executing service of controller.
 * }
 */
TEST_F(ModbusAdapterStoTest, testRemoveIsExecutingService)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  EXPECT_CLEARANCE;

  manipulator_.shutdownIsExecutingService();
  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("unhold_callback");

  EXPECT_CALL(manipulator_, holdCb(_,_)).Times(1);
  EXPECT_CALL(manipulator_, haltCb(_,_)).WillOnce(ACTION_OPEN_BARRIER("halt_callback"));

  pub_.publish(createDefaultStoModbusMsg(STO_ACTIVE));

  BARRIER("halt_callback");

  EXPECT_CLEARANCE;

  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("unhold_callback");
}

/**
 * @brief Tests that a message giving sto request is handled correctly.
 * Controller must be holded and driver must be disabled.
 *
 * @tests{Stop1_Trigger,
 *  Test that Stop 1 is triggered if STO value changes to false.
 * }
 *
 */
TEST_F(ModbusAdapterStoTest, testHoldMsg)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  EXPECT_CLEARANCE;

  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("unhold_callback");

  EXPECT_STOP1;

  pub_.publish(createDefaultStoModbusMsg(STO_ACTIVE));

  BARRIER("halt_callback");
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that a message indicating a disconnect from modbus stops the
 *  robot even if the content would give sto clearance.
 * }
 */
TEST_F(ModbusAdapterStoTest, testDisconnectNoStoMsg)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  EXPECT_CLEARANCE;

  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("unhold_callback");

  EXPECT_STOP1;

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_CLEAR);
  msg->disconnect.data = true;

  pub_.publish(msg);

  BARRIER("halt_callback");
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that a message indicating a disconnect from modbus stops the robot
 *  when the msg itself would also require sto to go active.
 * }
 *
 */
TEST_F(ModbusAdapterStoTest, testDisconnectWithStoMsg)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  EXPECT_CLEARANCE;

  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("unhold_callback");

  EXPECT_STOP1;

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->disconnect.data = true;

  pub_.publish(msg);

  BARRIER("halt_callback");
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that a message indicating a disconnect from modbus stops with no
 *  other data defined in the message.
 * }
 */
TEST_F(ModbusAdapterStoTest, testDisconnectPure)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  EXPECT_CLEARANCE;

  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("unhold_callback");

  EXPECT_STOP1;

  ModbusMsgInStampedPtr msg (new ModbusMsgInStamped());
  msg->header.stamp = ros::Time::now();
  msg->disconnect.data = true;

  pub_.publish(msg);

  BARRIER("halt_callback");
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that stop happens if no version is defined.
 * }
 */
TEST_F(ModbusAdapterStoTest, testNoVersion)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  EXPECT_CLEARANCE;

  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("unhold_callback");

  EXPECT_STOP1;

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->holding_registers.data.pop_back();

  pub_.publish(msg);

  BARRIER("halt_callback");
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that a stop happens if the version is wrong.
 * }
 */
TEST_F(ModbusAdapterStoTest, testWrongVersion)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  EXPECT_CLEARANCE;

  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("unhold_callback");

  EXPECT_STOP1;

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->holding_registers.data[1] = 0;

  pub_.publish(msg);

  BARRIER("halt_callback");
}


/**
 * @tests{Stop1_Trigger,
 *  Tests that a stop happens if a version 1 is received.
 * }
 *
 * @note Version 1 had mistake in specification on the hardware therefore
 * not supported at all.
 */
TEST_F(ModbusAdapterStoTest, testVersion1)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  EXPECT_CLEARANCE;

  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("unhold_callback");

  EXPECT_STOP1;

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->holding_registers.data[1] = 1;

  pub_.publish(msg);

  BARRIER("halt_callback");
}


/**
 * @tests{Stop1_Trigger,
 *  Test that stop happends if no Sto is defined.
 * }
 */
TEST_F(ModbusAdapterStoTest, testNoSto)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  EXPECT_CLEARANCE;

  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("unhold_callback");

  EXPECT_STOP1;

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->holding_registers.data.erase(msg->holding_registers.data.begin());
  msg->holding_registers.layout.data_offset = test_api_spec.getRegisterDefinition(modbus_api_spec::VERSION);

  pub_.publish(msg);

  BARRIER("halt_callback");
}

/**
 * @brief Check construction of the exception (essentially for full function coverage)
 */
TEST_F(ModbusAdapterStoTest, ModbusMsgExceptionCTOR)
{
  ModbusMsgStoWrapperException* exception = new ModbusMsgStoWrapperException("test");

  delete exception;
}

/**
 * @tests{STO_false_during_recover,
 *  Tests that the driver is halted upon completion of recover and no controller service is called
 *  if STO changes to false during recover.
 * }
 */
TEST_F(ModbusAdapterStoTest, testStoChangeDuringRecover)
{
  manipulator_.advertiseServices(nh_, HOLD_SERVICE_T, UNHOLD_SERVICE_T, HALT_SERVICE_T, RECOVER_SERVICE_T,
                                 IS_EXECUTING_SERVICE_T);

  modbus_sto_adapter_.reset(new ModbusAdapterSto(nh_, test_api_spec));

  // define function for recover-invoke action
  std::function<bool()> recover_action = [this]() {
    this->triggerClearEvent("recover_callback");
    this->pub_.publish(createDefaultStoModbusMsg(STO_ACTIVE));
    return true;
  };

  EXPECT_CALL(manipulator_, holdCb(_,_)).Times(0);
  EXPECT_CALL(manipulator_, isExecutingCb(_,_)).Times(0);
  EXPECT_CALL(manipulator_, unholdCb(_,_)).Times(0);
  EXPECT_CALL(manipulator_, haltCb(_,_)).Times(1).WillOnce(ACTION_OPEN_BARRIER("halt_callback"));
  EXPECT_CALL(manipulator_, recoverCb(_,_)).Times(1).WillOnce(InvokeWithoutArgs(recover_action));

  pub_.publish(createDefaultStoModbusMsg(STO_CLEAR));

  BARRIER("recover_callback");
  BARRIER("halt_callback");
}

} // namespace prbt_hardware_support

int main(int argc, char** argv)
{
  ros::init(argc, argv, "integrationtest_modbus_adapter_sto");
  ros::NodeHandle nh_;

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
