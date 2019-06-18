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

#ifndef MODBUS_ADAPTER_OPERATION_MODE_H
#define MODBUS_ADAPTER_OPERATION_MODE_H

#include <memory>

#include <ros/ros.h>
#include <message_filters/subscriber.h>

#include <prbt_hardware_support/adapter_operation_mode.h>

#include <prbt_hardware_support/operation_mode_filter.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/update_filter.h>
#include <prbt_hardware_support/modbus_msg_operation_mode_wrapper.h>
#include <prbt_hardware_support/modbus_api_spec.h>

namespace prbt_hardware_support
{

/**
 * @brief Listens to the modbus_read topic and offers a service
 * informing about the active operation mode
 */
class ModbusAdapterOperationMode : public AdapterOperationMode
{
public:
  ModbusAdapterOperationMode(ros::NodeHandle& nh, const ModbusApiSpec& api_spec);

private:
  void modbusInMsgCallback(const ModbusMsgInStampedConstPtr& msg);
  void internalMsgCallback(const ModbusMsgOperationModeWrapper& msg);

private:
  //! Filters consecutive messages with the same timestamp.
  //! Passed messages are redirected to the OperationModeFilter.
  std::shared_ptr< message_filters::UpdateFilter<ModbusMsgInStamped> > update_filter_;

  //! Filters messages with a change in the operation mode.
  //! Passed messages are redirected to modbusInMsgCallback().
  std::shared_ptr< message_filters::OperationModeFilter > operation_mode_filter_;

  //! Subscribes to TOPIC_MODBUS_READ and redirects received messages
  //! to the update-filter.
  std::shared_ptr< message_filters::Subscriber<ModbusMsgInStamped> > modbus_read_sub_;

  //! Currently valid api_spec (defines modbus register semantic)
  const ModbusApiSpec api_spec_;

};

} // namespace prbt_hardware_support
#endif // MODBUS_ADAPTER_OPERATION_MODE_H
