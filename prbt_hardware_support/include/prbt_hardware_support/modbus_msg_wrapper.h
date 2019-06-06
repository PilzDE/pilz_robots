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
#ifndef MODBUS_MSG_WRAPPER_H
#define MODBUS_MSG_WRAPPER_H

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/modbus_msg_wrapper_exception.h>

namespace prbt_hardware_support
{

/**
 * @brief Wrapper class to add semantic to a raw ModbusMsgInStamped
 *
 * Allows to easy access to the content behind a raw modbus message
 * which is assumed to contain data about the operation mode.
 */
class ModbusMsgWrapper
{
public:
  /**
   * @brief Construct a new Modbus Msg Wrapper object
   *
   * @throw ModbusMsgWrapperException if no version can be found in the message and thus interpretation is impossible
   */
  ModbusMsgWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw, const ModbusApiSpec& api_spec);

  /**
   * @return Get the API version defined in the Modbus message.
   */
  virtual unsigned int getVersion() const;

  /**
   * @brief Check if the Modbus message informs about a disconnect
   * from the server.
   *
   * @return true if the message informs about a disconnect, otherwise false.
   */
  virtual bool isDisconnect() const;

protected:

  /**
   * @brief Check if a certain holding register is define in the Modbus message.
   *
   * @returns true if the message has the register defined, otherwise false.
   */
  virtual bool hasRegister(const ModbusMsgInStampedConstPtr& modbus_msg_raw,
                           uint32_t reg) const;

  /**
   * @returns the content of the holding register.
   */
  virtual uint16_t getRegister(const ModbusMsgInStampedConstPtr& modbus_msg_raw,
                               uint32_t reg) const;

  /**
   * @brief Check if the modbus_msg contains the API version.
   */
  virtual bool hasVersion(const ModbusMsgInStampedConstPtr& modbus_msg_raw) const;

protected:
  ModbusMsgInStampedConstPtr msg_;
  const ModbusApiSpec api_spec_;
};

inline ModbusMsgWrapper::ModbusMsgWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw,
                                          const ModbusApiSpec& api_spec):
  msg_(modbus_msg_raw),
  api_spec_(api_spec)
{
  if (isDisconnect()) // TODO  @agu find a better way
  {
    // A disconnect message does not have to fullfill any requirements
    return;
  }

  if(!hasVersion(msg_))
  {
    throw ModbusMsgWrapperException("Received message does not contain a version.");
  }
}

inline bool ModbusMsgWrapper::hasRegister(const ModbusMsgInStampedConstPtr& modbus_msg_raw, uint32_t reg) const
{
  uint32_t relative_idx = reg - modbus_msg_raw->holding_registers.layout.data_offset;

  return modbus_msg_raw->holding_registers.data.size() > relative_idx;
}

inline uint16_t ModbusMsgWrapper::getRegister(const ModbusMsgInStampedConstPtr& modbus_msg_raw, uint32_t reg) const
{
  return modbus_msg_raw->holding_registers.data.at(reg - modbus_msg_raw->holding_registers.layout.data_offset);
}

inline bool ModbusMsgWrapper::hasVersion(const ModbusMsgInStampedConstPtr& modbus_msg_raw) const
{
  return hasRegister(modbus_msg_raw, api_spec_.getRegisterDefinition(modbus_api_spec::VERSION));
}

inline unsigned int ModbusMsgWrapper::getVersion() const
{
  return getRegister(msg_, api_spec_.getRegisterDefinition(modbus_api_spec::VERSION));
}

inline bool ModbusMsgWrapper::isDisconnect() const
{
  return msg_->disconnect.data;
}

}

#endif // MODBUS_MSG_WRAPPER_H