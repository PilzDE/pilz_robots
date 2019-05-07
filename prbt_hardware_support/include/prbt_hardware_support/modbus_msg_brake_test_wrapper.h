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
#ifndef MODBUS_MSG_BRAKE_TEST_WRAPPER_H
#define MODBUS_MSG_BRAKE_TEST_WRAPPER_H

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/modbus_msg_brake_test_wrapper_exception.h>

namespace prbt_hardware_support
{

/**
 * @brief Wrapper class to add semantic to a raw ModbusMsgInStamped
 *
 * Allows to easy access to the content behind a raw modbus message
 * which is assumed to contain data about brake test.
 */
class ModbusMsgBrakeTestWrapper
{
public:
  ModbusMsgBrakeTestWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw, const ModbusApiSpec& api_spec);

  /**
   * @return Get the API version defined in the Modbus message.
   */
  unsigned int getVersion() const;

  /**
   * @brief Get the brake test required flag from the Modbus message.
   *
   * @return true if the a brake test is required, otherwise false.
   */
  bool isBrakeTestRequired() const;

  /**
   * @brief Check if the Modbus message informs about a disconnect
   * from the server.
   *
   * @return true if the message informs about a disconnect, otherwise false.
   */
  bool isDisconnect() const;

private:

  /**
   * @brief Check if a certain holding register is define in the Modbus message.
   *
   * @returns true if the message has the register defined, otherwise false.
   */
  bool hasRegister(const ModbusMsgInStampedConstPtr& modbus_msg_raw,
                          uint32_t reg) const;

  /**
   * @returns the content of the holding register.
   */
  uint16_t getRegister(const ModbusMsgInStampedConstPtr& modbus_msg_raw,
                              uint32_t reg) const;

  /**
   * @brief Check if the message contains a brake test required definition.
   *
   * @return true if a brake test required flag is defined, false otherwise.
   */
  bool hasBrakeTestRequiredFlag(const ModbusMsgInStampedConstPtr& modbus_msg_raw) const;

  /**
   * @brief Check if the modbus_msg contains the API version.
   */
  bool hasVersion(const ModbusMsgInStampedConstPtr& modbus_msg_raw) const;

private:
  ModbusMsgInStampedConstPtr msg_;
  const ModbusApiSpec api_spec_;
};

inline ModbusMsgBrakeTestWrapper::ModbusMsgBrakeTestWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw,
                                                            const ModbusApiSpec& api_spec):
  msg_(modbus_msg_raw),
  api_spec_(api_spec)
{
  if (isDisconnect())
  {
    // A disconnect message does not have to fullfill any requirements
    return;
  }

  if(!hasVersion(msg_))
  {
    throw ModbusMsgBrakeTestWrapperException("Received message does not contain a version.");
  }

  if(!hasBrakeTestRequiredFlag(msg_))
  {
    throw ModbusMsgBrakeTestWrapperException("Received message does not contain a brake test status.");
  }
}

inline bool ModbusMsgBrakeTestWrapper::hasRegister(const ModbusMsgInStampedConstPtr& modbus_msg_raw, uint32_t reg) const
{
  uint32_t relative_idx = reg - modbus_msg_raw->holding_registers.layout.data_offset;

  return modbus_msg_raw->holding_registers.data.size() > relative_idx;
}

inline uint16_t ModbusMsgBrakeTestWrapper::getRegister(const ModbusMsgInStampedConstPtr& modbus_msg_raw, uint32_t reg) const
{
  return modbus_msg_raw->holding_registers.data.at(reg - modbus_msg_raw->holding_registers.layout.data_offset);
}

inline bool ModbusMsgBrakeTestWrapper::hasVersion(const ModbusMsgInStampedConstPtr& modbus_msg_raw) const
{
  return hasRegister(modbus_msg_raw, api_spec_.version_register_);
}

inline unsigned int ModbusMsgBrakeTestWrapper::getVersion() const
{
  return getRegister(msg_, api_spec_.version_register_);
}

inline bool ModbusMsgBrakeTestWrapper::hasBrakeTestRequiredFlag(const ModbusMsgInStampedConstPtr& modbus_msg_raw) const
{
  return hasRegister(modbus_msg_raw, api_spec_.braketest_register_);
}

inline bool ModbusMsgBrakeTestWrapper::isBrakeTestRequired() const
{
  return getRegister(msg_, api_spec_.braketest_register_);
}

inline bool ModbusMsgBrakeTestWrapper::isDisconnect() const
{
  return msg_->disconnect.data;
}


}

#endif // MODBUS_MSG_BRAKE_TEST_WRAPPER_H
