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

#include <prbt_hardware_support/modbus_adapter_operation_mode.h>

#include <prbt_hardware_support/modbus_topic_definitions.h>


namespace prbt_hardware_support
{

static constexpr int DEFAULT_QUEUE_SIZE_MODBUS {1};

ModbusAdapterOperationMode::ModbusAdapterOperationMode(ros::NodeHandle& nh, const ModbusApiSpec& api_spec)
  : AdapterOperationMode(nh),
    api_spec_(api_spec)
{
  modbus_read_sub_ = std::make_shared< message_filters::Subscriber<ModbusMsgInStamped> >(nh, TOPIC_MODBUS_READ, DEFAULT_QUEUE_SIZE_MODBUS);
  update_filter_ = std::make_shared< message_filters::UpdateFilter<ModbusMsgInStamped> >(*modbus_read_sub_);
  operation_mode_filter_ = std::make_shared< message_filters::OperationModeFilter >(*update_filter_, api_spec_);
	operation_mode_filter_->registerCallback(boost::bind(&ModbusAdapterOperationMode::modbusInMsgCallback, this, _1));
}

void ModbusAdapterOperationMode::internalMsgCallback(const ModbusMsgOperationModeWrapper& msg)
{
  updateOperationMode(msg.getOperationMode());
}

void ModbusAdapterOperationMode::modbusInMsgCallback(const ModbusMsgInStampedConstPtr& msg_raw)
{
	/* The ModbusMsgBrakeTestWrapperException will be handled in the BrakeTestFilter */
	ModbusMsgOperationModeWrapper msg(msg_raw, api_spec_);
	internalMsgCallback(msg);
}

}
