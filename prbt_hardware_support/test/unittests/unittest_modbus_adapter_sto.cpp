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
#include <memory>
#include <functional>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/param_names.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_msg_sto_wrapper.h>
#include <prbt_hardware_support/modbus_msg_in_builder.h>
#include <prbt_hardware_support/modbus_adapter_sto.h>
#include <prbt_hardware_support/register_container.h>
#include <prbt_hardware_support/wait_for_topic.h>

namespace prbt_hardware_support
{

static constexpr bool STO_CLEAR {true};
static constexpr bool STO_ACTIVE {false};

static const ModbusApiSpec TEST_API_SPEC{ {modbus_api_spec::VERSION, 513},
                                          {modbus_api_spec::STO, 512} };

static constexpr int MODBUS_API_VERSION_FOR_TESTING {2};

using namespace prbt_hardware_support;

using std::placeholders::_1;


class ModbusAdapterStoTest : public ::testing::Test
{
public:
  ModbusMsgInStampedPtr createDefaultStoModbusMsg(bool sto_clear);

public:
  MOCK_METHOD1(sendStoUpdate, void(const bool sto));

};

ModbusMsgInStampedPtr ModbusAdapterStoTest::createDefaultStoModbusMsg(bool sto)
{
  static int msg_time_counter {1};
  RegCont tab_reg(TEST_API_SPEC.size());
  tab_reg[0] = sto;
  tab_reg[1] = MODBUS_API_VERSION_FOR_TESTING;
  ModbusMsgInStampedPtr msg {ModbusMsgInBuilder::createDefaultModbusMsgIn(TEST_API_SPEC.getMinRegisterDefinition(), tab_reg)};
  msg->header.stamp = ros::Time(msg_time_counter++);
  return msg;
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
  std::shared_ptr<ModbusMsgStoWrapper> ex {new ModbusMsgStoWrapper(msg_const_ptr, TEST_API_SPEC)};
}

/**
 * @brief Tests that a modbus message giving sto clearance leads
 * to STO clear message
 *
 */
TEST_F(ModbusAdapterStoTest, testSTOClearMsg)
{
  EXPECT_CALL(*this, sendStoUpdate(STO_CLEAR)).Times(1);

  ModbusAdapterSto sto_adapter {ModbusAdapterSto(std::bind(&ModbusAdapterStoTest::sendStoUpdate, this, _1), TEST_API_SPEC)};
  sto_adapter.modbusMsgCallback( createDefaultStoModbusMsg(STO_CLEAR) );
}

/**
 * @brief Tests that a modbus message giving STO leads to STO active message
 *
 */
TEST_F(ModbusAdapterStoTest, testSTOActiveMsg)
{
  EXPECT_CALL(*this, sendStoUpdate(STO_ACTIVE)).Times(1);

  ModbusAdapterSto sto_adapter {ModbusAdapterSto(std::bind(&ModbusAdapterStoTest::sendStoUpdate, this, _1), TEST_API_SPEC)};
  sto_adapter.modbusMsgCallback( createDefaultStoModbusMsg(STO_ACTIVE) );
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that a modbus message indicating a disconnect from modbus leads
 *  to STO active message even if the recieved modbus message content would give
 *  sto clearance.
 * }
 */
TEST_F(ModbusAdapterStoTest, testDisconnectNoStoMsg)
{
  EXPECT_CALL(*this, sendStoUpdate(STO_ACTIVE)).Times(1);

  ModbusAdapterSto sto_adapter {ModbusAdapterSto(std::bind(&ModbusAdapterStoTest::sendStoUpdate, this, _1), TEST_API_SPEC)};

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_CLEAR);
  msg->disconnect.data = true;
  sto_adapter.modbusMsgCallback( msg );
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that a modbus message indicating a disconnect from modbus leads
 *  to STO active message if the received the modbus message itself
 *  would also require sto to go active.
 * }
 *
 */
TEST_F(ModbusAdapterStoTest, testDisconnectWithStoMsg)
{
  EXPECT_CALL(*this, sendStoUpdate(STO_ACTIVE)).Times(1);

  ModbusAdapterSto sto_adapter {ModbusAdapterSto(std::bind(&ModbusAdapterStoTest::sendStoUpdate, this, _1), TEST_API_SPEC)};

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->disconnect.data = true;
  sto_adapter.modbusMsgCallback( msg );
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that a modbus message indicating a disconnect from modbus with no
 *  other data defined in the modbus message leads to STO active message
 * }
 */
TEST_F(ModbusAdapterStoTest, testDisconnectPure)
{
  EXPECT_CALL(*this, sendStoUpdate(STO_ACTIVE)).Times(1);

  ModbusAdapterSto sto_adapter {ModbusAdapterSto(std::bind(&ModbusAdapterStoTest::sendStoUpdate, this, _1), TEST_API_SPEC)};

  ModbusMsgInStampedPtr msg (new ModbusMsgInStamped());
  ros::Time::init();
  msg->header.stamp = ros::Time::now();
  msg->disconnect.data = true;
  sto_adapter.modbusMsgCallback( msg );
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that STO active message is send if no version is defined in modbus
 *  message.
 * }
 */
TEST_F(ModbusAdapterStoTest, testNoVersion)
{
  EXPECT_CALL(*this, sendStoUpdate(STO_ACTIVE)).Times(1);

  ModbusAdapterSto sto_adapter {ModbusAdapterSto(std::bind(&ModbusAdapterStoTest::sendStoUpdate, this, _1), TEST_API_SPEC)};

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->holding_registers.data.pop_back();
  sto_adapter.modbusMsgCallback( msg );
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that STO active message is send if the modbus message contains
 *  a incorrect version number.
 * }
 */
TEST_F(ModbusAdapterStoTest, testWrongVersion)
{
  EXPECT_CALL(*this, sendStoUpdate(STO_ACTIVE)).Times(1);

  ModbusAdapterSto sto_adapter {ModbusAdapterSto(std::bind(&ModbusAdapterStoTest::sendStoUpdate, this, _1), TEST_API_SPEC)};

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->holding_registers.data[1] = 0;
  sto_adapter.modbusMsgCallback( msg );
}


/**
 * @tests{Stop1_Trigger,
 *  Tests that STO active message is send if the modbus message contains
 *  version 1.
 * }
 *
 * @note Version 1 had mistake in specification on the hardware therefore
 * not supported at all.
 */
TEST_F(ModbusAdapterStoTest, testVersion1)
{
  EXPECT_CALL(*this, sendStoUpdate(STO_ACTIVE)).Times(1);

  ModbusAdapterSto sto_adapter {ModbusAdapterSto(std::bind(&ModbusAdapterStoTest::sendStoUpdate, this, _1), TEST_API_SPEC)};

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->holding_registers.data[1] = 1;
  sto_adapter.modbusMsgCallback( msg );
}


/**
 * @tests{Stop1_Trigger,
 *  Test that STO active message is send if the modbus message does not
 *  define/contain the STO flag.
 * }
 */
TEST_F(ModbusAdapterStoTest, testNoSto)
{
  EXPECT_CALL(*this, sendStoUpdate(STO_ACTIVE)).Times(1);

  ModbusAdapterSto sto_adapter {ModbusAdapterSto(std::bind(&ModbusAdapterStoTest::sendStoUpdate, this, _1), TEST_API_SPEC)};

  ModbusMsgInStampedPtr msg = createDefaultStoModbusMsg(STO_ACTIVE);
  msg->holding_registers.data.erase(msg->holding_registers.data.begin());
  msg->holding_registers.layout.data_offset = TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::VERSION);
  sto_adapter.modbusMsgCallback( msg );
}

/**
 * @brief Check construction of the exception (essentially for full function coverage)
 */
TEST_F(ModbusAdapterStoTest, ModbusMsgExceptionCTOR)
{
  std::shared_ptr<ModbusMsgStoWrapperException> exception_ptr{new ModbusMsgStoWrapperException("test")};
}

} // namespace prbt_hardware_support

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
