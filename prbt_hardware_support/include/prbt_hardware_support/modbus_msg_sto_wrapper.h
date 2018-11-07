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

#ifndef MODBUS_MSG_STO_WRAPPER_H
#define MODBUS_MSG_STO_WRAPPER_H

#include <prbt_hardware_support/modbus_api_definitions.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_msg_sto_wrapper_exception.h>

namespace prbt_hardware_support
{

/**
 * @brief Wrapper class to add semantic to a raw ModbusMsgInStamped
 *
 * Allows to easy access to the content behind a raw modbus message which is assumed to contain
 * data about STO clearance.
 */
class ModbusMsgStoWrapper
{
public:
  ModbusMsgStoWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw);

  /**
   * @brief Get the API version defined in the Modbus message
   *
   * @return unsigned int the version
   */
  unsigned int getVersion() const;

  /**
   * @brief Get the STO from the Modbus message
   *
   * @return true if the STO is active (manipulator should stop)
   * @return false if the STO is clear (manipulator can move)
   */
  bool getSTO() const;

  /**
   * @brief Check if the Modbus message informs about a disconnect from the server
   *
   * @return true if the message informs about a disconnect
   * @return false if there message does not inform about a disconnect
   */
  bool isDisconnect() const;

private:

  /**
   * @brief Check if a certain holding register is define in the Modbus message
   *
   * @param modbus_msg_raw the message to be checked
   * @param reg the register to be checked for
   * @return true if the message has the register defined
   * @return false if there is no such register defined in the message
   */
  bool hasRegister(const ModbusMsgInStampedConstPtr& modbus_msg_raw, uint32_t reg) const;

  /**
   * @brief Get the content of the holding register
   *
   * @param modbus_msg_raw the message
   * @param reg the register to retrieve the value from
   * @return value of the register
   */
   uint16_t getRegister(const ModbusMsgInStampedConstPtr& modbus_msg_raw, uint32_t reg) const;


  /**
   * @brief Check if the message contains a STO definition
   *
   * @param modbus_msg_raw the message to be checked
   * @return true if a STO is defined
   * @return false if there is no STO defined in the message
   */
  bool hasSTO(const ModbusMsgInStampedConstPtr& modbus_msg_raw) const;

  /**
   * @brief Check if the modbus_msg contains the API version
   *
   * @param modbus_msg_raw the message
   * @return true if a version is defined
   * @return false if no version is defined
   */
  bool hasVersion(const ModbusMsgInStampedConstPtr& modbus_msg_raw) const;

  ModbusMsgInStampedConstPtr msg_;
};


inline bool ModbusMsgStoWrapper::hasRegister(const ModbusMsgInStampedConstPtr& modbus_msg_raw, uint32_t reg) const
{
  uint32_t relative_idx = reg - modbus_msg_raw->holding_registers.layout.data_offset;

  return modbus_msg_raw->holding_registers.data.size() > relative_idx;
}

inline uint16_t ModbusMsgStoWrapper::getRegister(const ModbusMsgInStampedConstPtr& modbus_msg_raw, uint32_t reg) const
{
  return msg_->holding_registers.data.at(reg - msg_->holding_registers.layout.data_offset);
}

inline bool ModbusMsgStoWrapper::hasVersion(const ModbusMsgInStampedConstPtr& modbus_msg_raw) const
{
  return hasRegister(modbus_msg_raw, modbus_api::MODBUS_REGISTER_API);
}

inline unsigned int ModbusMsgStoWrapper::getVersion() const
{
  return getRegister(msg_, modbus_api::MODBUS_REGISTER_API);
}

inline bool ModbusMsgStoWrapper::hasSTO(const ModbusMsgInStampedConstPtr& modbus_msg_raw) const
{
  return hasRegister(modbus_msg_raw, modbus_api::v2::MODBUS_REGISTER_STO);
}

inline bool ModbusMsgStoWrapper::getSTO() const
{
  return getRegister(msg_, modbus_api::v2::MODBUS_REGISTER_STO);
}

inline bool ModbusMsgStoWrapper::isDisconnect() const
{
  return msg_->disconnect.data;
}

}

#endif // MODBUS_MSG_STO_WRAPPER_H