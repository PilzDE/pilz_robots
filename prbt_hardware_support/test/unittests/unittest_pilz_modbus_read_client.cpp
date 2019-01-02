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

#include <thread>
#include <memory>
#include <chrono>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <modbus/modbus.h>

#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/pilz_modbus_client_mock.h>
#include <prbt_hardware_support/pilz_modbus_read_client.h>
#include <prbt_hardware_support/pilz_modbus_exceptions.h>
#include <prbt_hardware_support/pilz_modbus_read_client_exception.h>

#include <prbt_hardware_support/client_tests_common.h>

#include <pilz_testutils/async_test.h>

namespace pilz_modbus_read_client_test
{
using ::testing::_;
using ::testing::AtLeast;
using ::testing::Return;
using ::testing::Throw;
using ::testing::InSequence;
using ::testing::InvokeWithoutArgs;
using namespace prbt_hardware_support;

static constexpr unsigned int DEFAULT_MODBUS_PORT_TEST {502};
static constexpr unsigned int REGISTER_FIRST_IDX_TEST {512};
static constexpr unsigned int REGISTER_SIZE_TEST {2};

static constexpr double WAIT_FOR_START_TIMEOUT_S {3.0};
static constexpr double WAIT_SLEEPTIME_S {0.1};
static constexpr double WAIT_FOR_STOP_TIMEOUT_S {3.0};

/**
 * @brief Test if PilzModbusReadClient correctly publishes ROS-Modbus messages.
 *
 * It subscribes to the topic of the test object with a mocked callback function.
 * The actual client needed by the test object is replaced by a mock object.
 */
class PilzModbusReadClientTests : public testing::Test, public testing::AsyncTest
{
public:
  virtual void SetUp();

protected:
  ros::Subscriber subscriber_;
  ros::AsyncSpinner spinner_{2};
  ros::NodeHandle nh_;

  MOCK_METHOD1(modbus_read_cb,  void(ModbusMsgInStamped msg));
};

void PilzModbusReadClientTests::SetUp()
{
  subscriber_ = nh_.subscribe<ModbusMsgInStamped>(prbt_hardware_support::TOPIC_MODBUS_READ, 1, &PilzModbusReadClientTests::modbus_read_cb, this);
  spinner_.start();
}

MATCHER_P(IsSuccessfullRead, vec, "") { return arg.holding_registers.data == vec; }
MATCHER(IsDisconnect, "") { return arg.disconnect.data; }


/**
 * @brief Test that PilzModbusReadClient initializes its client properly
 */
TEST_F(PilzModbusReadClientTests, testInitialization)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_,_))
      .Times(1)
      .WillOnce(Return(true));

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock));

  EXPECT_TRUE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));
}

/**
 * @brief Test that initialization is repeated if it does not work at first try
 */
TEST_F(PilzModbusReadClientTests, testInitializationWithRetry)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_,_))
      .Times(2)
      .WillOnce(Return(false))
      .WillOnce(Return(true));

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock));

  EXPECT_TRUE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST, REGISTER_SIZE_TEST, ros::Duration(0.1)));
}

/**
 * @brief Test that double initialization fails on seconds try
 */
TEST_F(PilzModbusReadClientTests, doubleInitialization)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_,_))
      .Times(1)
      .WillOnce(Return(true));

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock));

  EXPECT_TRUE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));
  EXPECT_FALSE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));
}

/**
 * @brief Test the initialization of the modbus read client if init fails.
 */
TEST_F(PilzModbusReadClientTests, failingInitialization)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_,_))
      .Times(1)
      .WillOnce(Return(false));

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock));

  EXPECT_FALSE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));
}

/**
 * @brief Test that repeated failed initialization yields in a failed init for the caller
 */
TEST_F(PilzModbusReadClientTests, failingInitializationWithRetry)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_,_))
      .Times(2)
      .WillOnce(Return(false))
      .WillOnce(Return(false));

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock));

  EXPECT_FALSE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST, REGISTER_SIZE_TEST, ros::Duration(0.1)));
}

/**
 * @brief Test that PilzModbusReadClient properly reads from the registers and reacts to disconnects
 */
TEST_F(PilzModbusReadClientTests, properReadingAndDisconnect)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  {
    InSequence s;

    EXPECT_CALL(*mock, init(_,_))
        .Times(1)
        .WillOnce(Return(true));

    EXPECT_CALL(*mock, readHoldingRegister(_,_))
        .WillOnce(Return(std::vector<uint16_t>{1, 2}))
        .WillOnce(Return(std::vector<uint16_t>{1, 2}))
        .WillOnce(Return(std::vector<uint16_t>{3, 4}))
        .WillOnce(Throw(ModbusExceptionDisconnect("disconnect_message")));
  }
  {
    InSequence s;
    EXPECT_CALL(*this, modbus_read_cb(IsSuccessfullRead(std::vector<uint16_t>{1, 2})))
        .Times(2);

    EXPECT_CALL(*this, modbus_read_cb(IsSuccessfullRead(std::vector<uint16_t>{3, 4})))
        .Times(1);

    EXPECT_CALL(*this, modbus_read_cb(IsDisconnect()))
        .Times(1)
        .WillOnce(ACTION_OPEN_BARRIER_VOID("disconnected"));
  }

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock));

  EXPECT_TRUE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));
  EXPECT_NO_THROW(client.run());
  BARRIER("disconnected");
}

/**
 * @brief Try to run the modbus read client without a foregoing call to init()
 */
TEST_F(PilzModbusReadClientTests, runningWithoutInit)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_,_)).Times(0);
  EXPECT_CALL(*mock, readHoldingRegister(_,_)).Times(0);
  EXPECT_CALL(*this, modbus_read_cb(_)).Times(0);

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock));

  EXPECT_THROW(client.run(), PilzModbusReadClientException);
}

/**
 * @brief Test that a running modbus read client can be stopped with the terminate method
 */
TEST_F(PilzModbusReadClientTests, terminateRunningClient)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_,_))
    .Times(1)
    .WillOnce(Return(true));
  ON_CALL(*mock, readHoldingRegister(_,_)).WillByDefault(Return(std::vector<uint16_t>{3, 4}));
  ON_CALL(*this, modbus_read_cb(IsSuccessfullRead(std::vector<uint16_t>{3,4})))
    .WillByDefault(Return());

  auto client = std::make_shared< PilzModbusReadClient >(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock));

  EXPECT_TRUE(client->init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));

  auto running_thread = std::thread([client]
  {
    client->run();
  });

  ros::Time start_waiting = ros::Time::now();
  while (!client->isRunning())
  {
    ros::Duration(WAIT_SLEEPTIME_S).sleep();
    if (ros::Time::now() > start_waiting + ros::Duration(WAIT_FOR_START_TIMEOUT_S))
    {
      break;
    }
  }
  EXPECT_TRUE(client->isRunning());

  client->terminate();

  start_waiting = ros::Time::now();
  while (client->isRunning())
  {
    ros::Duration(WAIT_SLEEPTIME_S).sleep();
    if (ros::Time::now() > start_waiting + ros::Duration(WAIT_FOR_STOP_TIMEOUT_S))
    {
      break;
    }
  }
  EXPECT_FALSE(client->isRunning());
  running_thread.join();
}

}  // namespace pilz_modbus_read_client_test

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pilz_modbus_read_client_test");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
