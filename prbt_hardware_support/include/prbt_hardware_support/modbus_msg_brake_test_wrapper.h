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
#include <prbt_hardware_support/modbus_msg_wrapper.h>
#include <prbt_hardware_support/modbus_msg_brake_test_wrapper_exception.h>

namespace prbt_hardware_support
{

/**
 * @brief Wrapper class to add semantic to a raw ModbusMsgInStamped
 *
 * Allows to easy access to the content behind a raw modbus message
 * which is assumed to contain data about brake test.
 */
class ModbusMsgBrakeTestWrapper : public ModbusMsgWrapper
{
public:
  ModbusMsgBrakeTestWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw, const ModbusApiSpec& api_spec);

  /**
   * @brief Get the brake test required flag from the Modbus message.
   *
   * @return true if the a brake test is required, otherwise false.
   */
  bool isBrakeTestRequired() const;

private:

  /**
   * @brief Check if the message contains a brake test required definition.
   *
   * @return true if a brake test required flag is defined, false otherwise.
   */
  bool hasBrakeTestRequiredFlag() const;
};

inline ModbusMsgBrakeTestWrapper::ModbusMsgBrakeTestWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw,
                                                            const ModbusApiSpec& api_spec):
ModbusMsgWrapper(modbus_msg_raw, api_spec)
{
  if(!hasBrakeTestRequiredFlag())
  {
    throw ModbusMsgBrakeTestWrapperException("Received message does not contain a brake test status.");
  }
}

inline bool ModbusMsgBrakeTestWrapper::hasBrakeTestRequiredFlag() const
{
  return hasRegister(api_spec_.getRegisterDefinition(modbus_api_spec::BRAKETEST_REQUEST));
}

inline bool ModbusMsgBrakeTestWrapper::isBrakeTestRequired() const
{
  return getRegister(api_spec_.getRegisterDefinition(modbus_api_spec::BRAKETEST_REQUEST));
}

}

#endif // MODBUS_MSG_BRAKE_TEST_WRAPPER_H
