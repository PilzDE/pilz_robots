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


/**
 * @brief Create modbus msg given api version and operation mode.
 */
ModbusMsgInStampedPtr createDefaultOpModeModbusMsg(unsigned int operation_mode,
                                                    unsigned int modbus_api_version,
                                                    uint32_t operation_mode_index,
                                                    uint32_t version_index)
{
  uint32_t first_index_to_read{std::min(operation_mode_index, version_index)};
  uint32_t last_index_to_read{std::max(operation_mode_index, version_index)};
  static int msg_time_counter{1};
  std::vector<uint16_t> tab_reg(last_index_to_read - first_index_to_read + 1);
  tab_reg[version_index - first_index_to_read] = modbus_api_version;
  tab_reg[operation_mode_index - first_index_to_read] = operation_mode;
  ModbusMsgInStampedPtr msg{createDefaultModbusMsgIn(first_index_to_read, tab_reg)};
  msg->header.stamp = ros::Time(msg_time_counter++);
  return msg;
}

}

#endif // PRBT_HARDWARE_SUPPORT_MODBUS_MSG_IN_UTILS_H
