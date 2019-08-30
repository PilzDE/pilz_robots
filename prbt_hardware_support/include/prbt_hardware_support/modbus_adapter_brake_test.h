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

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/filter_pipeline.h>
#include <prbt_hardware_support/IsBrakeTestRequired.h>

namespace prbt_hardware_support
{

/**
 * @brief Listens to the modbus_read topic and publishes a message
 * informing about a required brake test.
 */
class ModbusAdapterBrakeTest
{
public:
  ModbusAdapterBrakeTest(ros::NodeHandle& nh, const ModbusApiSpec& api_spec);

private:
  /**
   * @brief Called whenever a new modbus message arrives.
   *
   * @note Filters like for example the UpdateFilter can restrict
   * the number of incoming messages.
   */
  void modbusMsgCallback(const ModbusMsgInStampedConstPtr& msg_raw);

  void updateBrakeTestRequiredState(IsBrakeTestRequiredResponse::_result_type brake_test_required);

  /**
   * @brief Stores the brake test required flag and
   * initializes the brake test service,
   * the first time the function is called.
   */
  bool isBrakeTestRequired(IsBrakeTestRequired::Request&,
                           IsBrakeTestRequired::Response& response);

private:
  const ModbusApiSpec api_spec_;
  std::unique_ptr<FilterPipeline> filter_pipeline_;

  using TBrakeTestRequired = IsBrakeTestRequiredResponse::_result_type;

  //! Store the current state of whether a brake test is required
  TBrakeTestRequired brake_test_required_ {IsBrakeTestRequiredResponse::UNKNOWN};

  //! Server serving a service to ask whether a brake test is currently required
  ros::ServiceServer is_brake_test_required_server_;

};

} // namespace prbt_hardware_support
#endif // MODBUS_ADAPTER_BRAKE_TEST_H
