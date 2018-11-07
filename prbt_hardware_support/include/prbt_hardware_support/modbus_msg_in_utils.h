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

#ifndef PRBT_HARDWARE_SUPPORT_MODBUS_MSG_IN_UTILS_H
#define PRBT_HARDWARE_SUPPORT_MODBUS_MSG_IN_UTILS_H

#include <std_msgs/MultiArrayDimension.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>

namespace prbt_hardware_support
{

static void setLayout(std_msgs::MultiArrayLayout* layout, const uint32_t& offset, const uint32_t& size)
{
  layout->data_offset = offset;
  layout->dim.push_back(std_msgs::MultiArrayDimension());
  layout->dim[0].size = size;
  layout->dim[0].stride = 1;
  layout->dim[0].label = "Data in holding register";
}

/**
 * @brief Creates a standard ModbusMsgIn which
 * contains default values for all essential
 * elements of the message.
 */
static ModbusMsgInStamped* createDefaultModbusMsgIn(const uint32_t& offset,
                                             const std::vector<uint16_t>& holding_register)
{
  ModbusMsgInStamped* msg { new ModbusMsgInStamped() };
  setLayout(&(msg->holding_registers.layout), offset, holding_register.size());

  msg->disconnect.data = false;
  msg->holding_registers.data = holding_register;

  return msg;
}

}

#endif // PRBT_HARDWARE_SUPPORT_MODBUS_MSG_IN_UTILS_H
