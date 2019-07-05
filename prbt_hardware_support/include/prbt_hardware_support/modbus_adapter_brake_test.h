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

#include <ros/ros.h>

#include <prbt_hardware_support/adapter_brake_test.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/filter_pipeline.h>

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
  virtual ~ModbusAdapterBrakeTest() = default;

private:
  /**
   * @brief Called whenever a new modbus message arrives.
   *
   * @note Filters like for example the UpdateFilter can restrict
   * the number of incoming messages.
   */
  void modbusMsgCallback(const ModbusMsgInStampedConstPtr& msg_raw);

private:
  const ModbusApiSpec api_spec_;
  std::unique_ptr<FilterPipeline> filter_pipeline_;

};

} // namespace prbt_hardware_support
#endif // MODBUS_ADAPTER_BRAKE_TEST_H
