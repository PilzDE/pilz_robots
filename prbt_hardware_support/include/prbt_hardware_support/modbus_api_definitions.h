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

#ifndef MODBUS_API_DEFINITIONS_H
#define MODBUS_API_DEFINITIONS_H

#include <string>

namespace prbt_hardware_support
{

namespace modbus_api
{
  // Must hold for every version
  static constexpr uint32_t MODBUS_REGISTER_API {513};

  namespace v1
  {
    static constexpr uint32_t MODBUS_REGISTER_STO {512};

    static constexpr uint16_t MODBUS_STO_CLEAR_VALUE  {0};
    static constexpr uint16_t MODBUS_STO_ACTIVE_VALUE {1};
  }
}

}
#endif // MODBUS_API_DEFINITIONS_H
