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

#ifndef MODBUS_API_SPEC_H
#define MODBUS_API_SPEC_H

#include <ros/ros.h>

namespace prbt_hardware_support
{

/**
 * @brief Specifies the meaning of the holding registers.
 *
 * Currently specifies in which registers version and braketest_request are defined.
 */
class ModbusApiSpec
{
public:

  constexpr ModbusApiSpec(unsigned int version_register, unsigned int braketest_register):
    version_register_(version_register),
    braketest_register_(braketest_register){};

  const unsigned int version_register_;
  const unsigned int braketest_register_;
};

} // namespace prbt_hardware_support
#endif // MODBUS_API_SPEC_H
