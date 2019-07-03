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

#include <limits>
#include <stdexcept>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/OperationModes.h>
#include <prbt_hardware_support/register_container.h>

#include <boost/numeric/conversion/cast.hpp>

namespace prbt_hardware_support
{

static void setDefaultLayout(std_msgs::MultiArrayLayout* layout,
                             const std_msgs::MultiArrayLayout::_data_offset_type& offset,
                             const RegCont::size_type& size)
{
  if (size > std::numeric_limits<std_msgs::MultiArrayDimension::_size_type>::max())
  {
    throw std::invalid_argument("Argument \"size\" must not exceed max value of type \"std_msgs::MultiArrayDimension::_size_type\"");
  }

  layout->data_offset = offset;
  layout->dim.push_back(std_msgs::MultiArrayDimension());
  layout->dim.back().size = static_cast<std_msgs::MultiArrayDimension::_size_type>(size);
  layout->dim.back().stride = 1;
  layout->dim.back().label = "Data in holding register";
}

/**
 * @brief Creates a standard ModbusMsgIn which contains default values for
 * all essential elements of the message.
 */
static ModbusMsgInStampedPtr createDefaultModbusMsgIn(const std_msgs::MultiArrayLayout::_data_offset_type& offset,
                                                      const RegCont& holding_register)
{
  ModbusMsgInStampedPtr msg { new ModbusMsgInStamped() };
  setDefaultLayout(&(msg->holding_registers.layout), offset, boost::numeric_cast<uint32_t>(holding_register.size()));
  msg->holding_registers.data = holding_register;
  msg->disconnect.data = false;

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
  tab_reg[version_index - first_index_to_read] = static_cast<uint16_t>(modbus_api_version);
  tab_reg[operation_mode_index - first_index_to_read] = static_cast<uint16_t>(operation_mode);
  ModbusMsgInStampedPtr msg{createDefaultModbusMsgIn(first_index_to_read, tab_reg)};
  msg->header.stamp = ros::Time(msg_time_counter++);
  return msg;
}

/**
 * @brief Help on construction for ModbusMsgIn Messages.
 *
 * Mainly intented for usage in tests.
 *
 */
class ModbusMsgInBuilder
{
public:
  ModbusMsgInBuilder(const ModbusApiSpec &api_spec):
    api_spec_(api_spec)
  {
    // Initialize for ROS time if not already initialized
    if(!ros::Time::isValid()){
      ros::Time::init();
    }
  }

  ModbusMsgInBuilder& setSto(uint16_t sto){
    setRegister(api_spec_.getRegisterDefinition(modbus_api_spec::STO), sto);
    return *this;
  }

  ModbusMsgInBuilder& setOperationMode(uint16_t mode){
    setRegister(api_spec_.getRegisterDefinition(modbus_api_spec::OPERATION_MODE), mode);
    return *this;
  }

  ModbusMsgInBuilder& setApiVersion(uint16_t version){
    setRegister(api_spec_.getRegisterDefinition(modbus_api_spec::VERSION), version);
    return *this;
  }

  ModbusMsgInStampedPtr build()
  {
    // Get the minimum and maximum register
    uint32_t first_index_to_read{register_values_.cbegin()->first};
    uint32_t last_index_to_read{register_values_.crbegin()->first};

    // Setup vector
    RegCont tab_reg(last_index_to_read - first_index_to_read + 1);
    for(auto reg : register_values_)
    {
      tab_reg[reg.first - first_index_to_read] = reg.second;
    }

    ModbusMsgInStampedPtr msg { createDefaultModbusMsgIn(first_index_to_read, tab_reg) };
    msg->header.stamp = ros::Time::now();
    return msg;
  }

  inline void setRegister(unsigned int register_n, uint16_t value)
  {
    register_values_[register_n] = value;
  }

private:
  const ModbusApiSpec api_spec_;
  std::map<unsigned int, uint16_t> register_values_;
};

}

#endif // PRBT_HARDWARE_SUPPORT_MODBUS_MSG_IN_UTILS_H
