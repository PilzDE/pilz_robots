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
#include <prbt_hardware_support/modbus_msg_wrapper.h>

namespace prbt_hardware_support
{

/**
 * @brief Wrapper class to add semantic to a raw ModbusMsgInStamped
 *
 * Allows to easy access to the content behind a raw modbus message which is assumed to contain
 * data about STO clearance.
 */
class ModbusMsgStoWrapper : public ModbusMsgWrapper
{
public:
  ModbusMsgStoWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw, const ModbusApiSpec& api_spec);

  /**
   * @brief Calls ModbusMsgWrapper::checkStructuralIntegrity().
   *
   * @throw ModbusMsgStoWrapperException if STO register is missing.
   */
  virtual void checkStructuralIntegrity() const override;

  /**
   * @brief Get the STO from the Modbus message
   *
   * @return true if the STO is active (manipulator should stop)
   * @return false if the STO is clear (manipulator can move)
   */
  bool getSTO() const;

private:
  /**
   * @brief Check if the message contains a STO definition
   *
   * @return true if a STO is defined
   * @return false if there is no STO defined in the message
   */
  bool hasSTO() const;

};

inline bool ModbusMsgStoWrapper::hasSTO() const
{
  return hasRegister(getApiSpec().getRegisterDefinition(modbus_api_spec::STO));
}

inline bool ModbusMsgStoWrapper::getSTO() const
{
  return getRegister(getApiSpec().getRegisterDefinition(modbus_api_spec::STO));
}

}

#endif // MODBUS_MSG_STO_WRAPPER_H
