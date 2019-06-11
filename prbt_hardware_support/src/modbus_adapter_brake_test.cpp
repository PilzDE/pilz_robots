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

#include <prbt_hardware_support/modbus_adapter_brake_test.h>

#include <prbt_hardware_support/modbus_msg_brake_test_wrapper_exception.h>
#include <prbt_hardware_support/modbus_topic_definitions.h>


namespace prbt_hardware_support
{

static constexpr int DEFAULT_QUEUE_SIZE_MODBUS {1};

static constexpr unsigned int MODBUS_API_VERSION_REQUIRED {2};

ModbusAdapterBrakeTest::ModbusAdapterBrakeTest(ros::NodeHandle& nh, const ModbusApiSpec& api_spec)
  : AdapterBrakeTest(nh),
    api_spec_(api_spec)
{
  modbus_read_sub_ = std::make_shared< message_filters::Subscriber<ModbusMsgInStamped> >(nh, TOPIC_MODBUS_READ, DEFAULT_QUEUE_SIZE_MODBUS);
  update_filter_ = std::make_shared< message_filters::UpdateFilter<ModbusMsgInStamped> >(*modbus_read_sub_);
  brake_test_filter_ = std::make_shared< message_filters::BrakeTestFilter<ModbusMsgInStamped> >(*update_filter_, api_spec_);
	brake_test_filter_->registerCallback(boost::bind(&ModbusAdapterBrakeTest::modbusInMsgCallback, this, _1));
}

void ModbusAdapterBrakeTest::internalMsgCallback(const ModbusMsgBrakeTestWrapper& msg)
{
  if(msg.isDisconnect())
  {
    // In case of a disconnect nothing works anymore (see pilz_modbus_read_client.cpp)
    // and the sto node performs a stop.
    // Here, we do not publish anything.
    return;
  }

  unsigned int modbus_api_version = msg.getVersion();
  if(modbus_api_version != MODBUS_API_VERSION_REQUIRED)
  {
    ROS_ERROR_STREAM("Received Modbus Message with unsupported API Version "
                    << modbus_api_version << ", required Version is " << MODBUS_API_VERSION_REQUIRED);
    ROS_ERROR_STREAM("Can not determine from Modbus message if brake-test is required.");
    return;
  }

	updateBrakeTestRequiredState(msg.isBrakeTestRequired());
}

void ModbusAdapterBrakeTest::modbusInMsgCallback(const ModbusMsgInStampedConstPtr& msg_raw)
{
	/* The ModbusMsgBrakeTestWrapperException will be handled in the BrakeTestFilter */
	ModbusMsgBrakeTestWrapper msg(msg_raw, api_spec_);
	internalMsgCallback(msg);
}

}
