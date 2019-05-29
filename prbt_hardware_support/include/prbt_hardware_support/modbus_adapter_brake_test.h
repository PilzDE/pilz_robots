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

#ifndef MODBUS_ADAPTER_BRAKE_TEST_H
#define MODBUS_ADAPTER_BRAKE_TEST_H

#include <memory>
#include <string>

#include <ros/ros.h>
#include <message_filters/subscriber.h>

#include <prbt_hardware_support/adapter_brake_test.h>

#include <prbt_hardware_support/brake_test_filter.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/update_filter.h>
#include <prbt_hardware_support/modbus_msg_brake_test_wrapper.h>

namespace prbt_hardware_support
{

/**
 * @brief Listens to the modbus_read topic and publishes a message
 * informing about a required brake test.
 */
class ModbusAdapterBrakeTest : public AdapterBrakeTest
{
public:
  ModbusAdapterBrakeTest(ros::NodeHandle& nh, const ModbusApiSpec& api_spec);

private:
  void modbusInMsgCallback(const ModbusMsgInStampedConstPtr& msg);
  void internalMsgCallback(const ModbusMsgBrakeTestWrapper& msg);

private:
  //! Filters consecutive messages with the same timestamp.
  //! Passed messages are redirected to the brake-test-filter.
  std::shared_ptr< message_filters::UpdateFilter<ModbusMsgInStamped> > update_filter_;

  //! Filters messages with a change in brake test state.
  //! Passed messages are redirected to modbusInMsgCallback().
  std::shared_ptr< message_filters::BrakeTestFilter<ModbusMsgInStamped> > brake_test_filter_;

  //! Subscribes to TOPIC_MODBUS_READ and redirects received messages
  //! to the update-filter.
  std::shared_ptr< message_filters::Subscriber<ModbusMsgInStamped> > modbus_read_sub_;

  //! Currently valid api_spec (defines modbus register semantic)
  const ModbusApiSpec api_spec_;

};

} // namespace prbt_hardware_support
#endif // MODBUS_ADAPTER_BRAKE_TEST_H
