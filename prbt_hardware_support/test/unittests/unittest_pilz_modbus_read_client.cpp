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
#include <mutex>

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
using ::testing::Invoke;
using namespace prbt_hardware_support;

static constexpr unsigned int DEFAULT_MODBUS_PORT_TEST {502};
static constexpr unsigned int REGISTER_FIRST_IDX_TEST {512};
static constexpr unsigned int REGISTER_SIZE_TEST {2};
static constexpr unsigned int RESPONSE_TIMEOUT {10};

static constexpr double WAIT_FOR_START_TIMEOUT_S {3.0};
static constexpr double WAIT_SLEEPTIME_S {0.1};
static constexpr double WAIT_FOR_STOP_TIMEOUT_S {3.0};

/**
 * @brief Helper class to simplify the threading of the PilzModbusReadClient.
 */
class PilzModbusReadClientExecutor
{
public:
  PilzModbusReadClientExecutor(PilzModbusReadClient* client);

public:
  void start();
  void stop();

private:
  PilzModbusReadClient* client_;
  std::thread client_thread_;
};

PilzModbusReadClientExecutor::PilzModbusReadClientExecutor(PilzModbusReadClient* client)
  : client_(client)
{
}

void PilzModbusReadClientExecutor::start()
{
  client_thread_ = std::thread(&PilzModbusReadClient::run, client_);

  ros::Time start_waiting = ros::Time::now();
  while (!client_->isRunning())
  {
    ros::Duration(WAIT_SLEEPTIME_S).sleep();
    if (ros::Time::now() > start_waiting + ros::Duration(WAIT_FOR_START_TIMEOUT_S))
    {
      break;
    }
  }
}

void PilzModbusReadClientExecutor::stop()
{
  client_->terminate();
  ros::Time start_waiting = ros::Time::now();
  while (client_->isRunning())
  {
    ros::Duration(WAIT_SLEEPTIME_S).sleep();
    if (ros::Time::now() > start_waiting + ros::Duration(WAIT_FOR_STOP_TIMEOUT_S))
    {
      break;
    }
  }
  client_thread_.join();
}

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

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock),
                              RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ);

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

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock),
                              RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ);

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

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock),
                              RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ);

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

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock),
                              RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ);

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

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock),
                              RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ);

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

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock),
                              RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ);

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

  PilzModbusReadClient client(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock),
                              RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ);

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

  auto client = std::make_shared< PilzModbusReadClient >(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock),
                                                         RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ);

  EXPECT_TRUE(client->init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));

  PilzModbusReadClientExecutor executor(client.get());
  executor.start();
  EXPECT_TRUE(client->isRunning());
  executor.stop();
  EXPECT_FALSE(client->isRunning());
}

void callbackDummy(const ModbusMsgInStampedConstPtr& /*msg*/) {}

/**
 * @brief Tests that topic name can be changed.
 */
TEST_F(PilzModbusReadClientTests, testTopicNameChange)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  const std::string topic_name {"test_topic_name"};

  auto client = std::make_shared< PilzModbusReadClient >(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock),
                                                         RESPONSE_TIMEOUT, topic_name);
  // Wait for a moment to ensure the topic is "up and running"
  ros::Duration(WAIT_SLEEPTIME_S).sleep();
  ros::Subscriber sub = nh_.subscribe<ModbusMsgInStamped>(topic_name, 1, &callbackDummy);
  ASSERT_EQ(1u, sub.getNumPublishers());
}

/**
 * @brief Tests that response timeout can be changed.
 */
TEST_F(PilzModbusReadClientTests, testSettingOfTimeOut)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  const unsigned int response_timeout {RESPONSE_TIMEOUT + 4};

  EXPECT_CALL(*mock, init(_,_))
      .Times(1)
      .WillOnce(Return(true));

  EXPECT_CALL(*mock, setResponseTimeoutInMs(response_timeout)).Times(1);

  auto client = std::make_shared< PilzModbusReadClient >(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock),
                                                         response_timeout, prbt_hardware_support::TOPIC_MODBUS_READ);

  EXPECT_TRUE(client->init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));
}

/**
 * @brief Strictly increases the holding register each time the
 * "readHoldingRegister" method is called.
 */
class HoldingRegisterIncreaser
{
public:
  HoldingRegisterIncreaser(unsigned int register_index = 0u)
    : register_index_(register_index)
  {}

public:
  std::vector<uint16_t> readHoldingRegister(int /*addr*/, int nb)
  {
    std::vector<uint16_t> vec( static_cast<std::vector<uint16_t>::size_type>(nb) , 0u);
    vec.at(register_index_) = curr_count_;
    ++curr_count_;
    return vec;
  }

private:
  const unsigned int register_index_;
  uint16_t curr_count_ {0};
};

class RegisterBuffer
{
public:
  void add(ModbusMsgInStamped msg)
  {
    std::unique_lock<std::mutex> lk(m_);
    buffer_ = msg.holding_registers.data.at(0);
  }

  uint16_t get()
  {
    std::unique_lock<std::mutex> lk(m_);
    return buffer_;
  }

private:
  std::mutex m_;
  uint16_t buffer_;
};

/**
 * @brief Tests that the frequency with which the registers are read,
 * can be changed.
 *
 * This test does NOT precisly check (tolerance ca. 10Hz) if the exact
 * frequency could be set. It just checks that the frequency was
 * changed.
 *
 *
 * Test Sequence:
 * - 1. Choose low read frequency (f1). The last received "modbus_read" messages
 *      is stored in a buffer.
 * - 2. Choose "modbus_read" topic check frequency f2, with f2 slightly bigger than f1.
 * - 3. Start a loop (running exactly for #f1 iterations), which checks with
 *      frequency f2 the last received "modbus_read" message.
 * - 4. Stop all loops and the PilzModbusReadClient.
 *
 * Expected Results:
 * - 1. -
 * - 2. -
 * - 3. The last received "modbus_read" messges must fullfill the condition:
 *      "new_register_value = last_register_value + 1".
 *      If the condition does not hold, the read fequency could not be
 *      set properly and is probably to high.
 * - 4. Check that the number of received messages is creater than #f1. If the
 *      condition does not hold, the read fequency is not set properly
 *      and is probably to low.
 */
TEST_F(PilzModbusReadClientTests, testSettingReadFrequency)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_,_))
      .Times(1)
      .WillOnce(Return(true));

  HoldingRegisterIncreaser fake_holding_register;
  ON_CALL(*mock, readHoldingRegister(_, _))
      .WillByDefault(Invoke(&fake_holding_register, &HoldingRegisterIncreaser::readHoldingRegister));

  RegisterBuffer buffer;
  ON_CALL(*this, modbus_read_cb(_))
      .WillByDefault(Invoke(&buffer, &RegisterBuffer::add));

  const double expected_read_frequency {30.0};
  auto client = std::make_shared< PilzModbusReadClient >(nh_,REGISTER_SIZE_TEST,REGISTER_FIRST_IDX_TEST,std::move(mock),
                                                         RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ,
                                                         expected_read_frequency);

  EXPECT_TRUE(client->init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));

  PilzModbusReadClientExecutor executor(client.get());
  executor.start();
  EXPECT_TRUE(client->isRunning());

  // The "modbus_read" messages from the PilzModbusReadClient are checked
  // faster as the set read frequency, to ensure that
  // all messages from the PilzModbusReadClient are processed by the test.
  const double msg_check_frequency {1.2*expected_read_frequency};
  // The timeout indirectly defines how many messages are checked
  // for "correctness".
  const unsigned int N_TIMEOUT { static_cast<unsigned int>(msg_check_frequency) + 1u };
  uint16_t last {0};
  bool first_value_set {false};
  ros::Rate rate(msg_check_frequency);
  for(unsigned int counter = 0; (counter < N_TIMEOUT) && ros::ok(); ++counter )
  {
    rate.sleep();
    if (!first_value_set)
    {
      last = buffer.get();
      first_value_set = true;
      continue;
    }

    uint16_t curr_value = buffer.get();
    if (curr_value == last)
    {
      continue;
    }
    uint16_t expected_value = last + 1u;
    EXPECT_EQ(expected_value, curr_value) << "Frequency used by PilzModbusReadClient is probably too higher";
    last = curr_value;
  }

  EXPECT_GE(buffer.get(), static_cast<uint16_t>(expected_read_frequency)) << "Frequency used by PilzModbusReadClient is probably too lower";

  executor.stop();
  EXPECT_FALSE(client->isRunning());
}

}  // namespace pilz_modbus_read_client_test

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pilz_modbus_read_client_test");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
