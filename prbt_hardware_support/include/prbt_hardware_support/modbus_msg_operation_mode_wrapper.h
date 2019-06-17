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
#ifndef MODBUS_MSG_OPERATION_MODE_WRAPPER_H
#define MODBUS_MSG_OPERATION_MODE_WRAPPER_H

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/modbus_msg_wrapper.h>
#include <prbt_hardware_support/modbus_msg_operation_mode_wrapper_exception.h>
#include <prbt_hardware_support/operation_modes.h>

namespace prbt_hardware_support
{

/**
 * @brief Wrapper class to add semantic to a raw ModbusMsgInStamped
 *
 * Allows to easy access to the content behind a raw modbus message
 * which is assumed to contain data about the operation mode.
 */
class ModbusMsgOperationModeWrapper : public ModbusMsgWrapper
{
public:
  ModbusMsgOperationModeWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw, const ModbusApiSpec& api_spec);

  /**
   * @brief Get the operation mode field from the Modbus message.
   *
   * @return The current value of the operation mode
   */
  OperationMode getOperationMode() const;

private:

  /**
   * @brief Check if the message contains a operation mode definition.
   *
   * @return true if a operation mode is defined, false otherwise.
   */
  bool hasOperationMode() const;
};

inline ModbusMsgOperationModeWrapper::ModbusMsgOperationModeWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw,
                                                            const ModbusApiSpec& api_spec):
  ModbusMsgWrapper(modbus_msg_raw, api_spec)
{
  if(!hasVersion())
  {
    throw ModbusMsgOperationModeWrapperException("Received message does not contain a version.");
  }

  if(!hasOperationMode())
  {
    throw ModbusMsgOperationModeWrapperException("Received message does not contain information about the operation mode.");
  }
}

inline bool ModbusMsgOperationModeWrapper::hasOperationMode() const
{
  return hasRegister(api_spec_.getRegisterDefinition(modbus_api_spec::OPERATION_MODE));
}

inline OperationMode ModbusMsgOperationModeWrapper::getOperationMode() const
{
    switch(getRegister(api_spec_.getRegisterDefinition(modbus_api_spec::OPERATION_MODE))){
        case 0:
            return OperationMode::UNKNOWN;
        case 1:
            return OperationMode::T1;
        case 2:
            return OperationMode::T2;
        case 3:
            return OperationMode::AUTO;
        default:
            return OperationMode::UNKNOWN;
    }
}

}

#endif // MODBUS_MSG_OPERATION_MODE_WRAPPER_H
