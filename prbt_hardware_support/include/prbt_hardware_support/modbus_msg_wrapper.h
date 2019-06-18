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
 * which is assumed to contain at least data about the api version used.
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

  virtual ~ModbusMsgWrapper() = default;

  /**
   * @return Get the API version defined in the Modbus message.
   */
  unsigned int getVersion() const;

  /**
   * @brief Check if the Modbus message informs about a disconnect
   * from the server.
   *
   * @return true if the message informs about a disconnect, otherwise false.
   */
  bool isDisconnect() const;

protected:

  /**
   * @brief Check if a certain holding register is define in the Modbus message.
   *
   * @returns true if the message has the register defined, otherwise false.
   */
  bool hasRegister(uint32_t reg) const;

  /**
   * @returns the content of the holding register.
   */
  uint16_t getRegister(uint32_t reg) const;

  /**
   * @brief Check if the modbus_msg contains the API version.
   */
  bool hasVersion() const;

protected:
  const ModbusApiSpec api_spec_;

private:
  const ModbusMsgInStampedConstPtr msg_;

};

inline ModbusMsgWrapper::ModbusMsgWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw,
                                          const ModbusApiSpec& api_spec):
  api_spec_(api_spec),
  msg_(modbus_msg_raw)
{
  if (isDisconnect())
  {
    // A disconnect message does not have to fullfill any requirements
    return;
  }

  if(!hasVersion())
  {
    throw ModbusMsgWrapperException("Received message does not contain a version.");
  }
}

inline bool ModbusMsgWrapper::hasRegister(uint32_t reg) const
{
  uint32_t relative_idx = reg - msg_->holding_registers.layout.data_offset;

  return msg_->holding_registers.data.size() > relative_idx;
}

inline uint16_t ModbusMsgWrapper::getRegister(uint32_t reg) const
{
  return msg_->holding_registers.data.at(reg - msg_->holding_registers.layout.data_offset);
}

inline bool ModbusMsgWrapper::hasVersion() const
{
  return hasRegister(api_spec_.getRegisterDefinition(modbus_api_spec::VERSION));
}

inline unsigned int ModbusMsgWrapper::getVersion() const
{
  return getRegister(api_spec_.getRegisterDefinition(modbus_api_spec::VERSION));
}

inline bool ModbusMsgWrapper::isDisconnect() const
{
  return msg_->disconnect.data;
}

}

#endif // MODBUS_MSG_WRAPPER_H
