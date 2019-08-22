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

#ifndef ADAPTER_OPERATION_MODE_H
#define ADAPTER_OPERATION_MODE_H

#include <ros/ros.h>

#include <prbt_hardware_support/IsBrakeTestRequired.h>
#include <prbt_hardware_support/GetOperationMode.h>
#include <prbt_hardware_support/OperationModes.h>

namespace prbt_hardware_support
{

/**
 * @brief Offers a service with information on the active operation mode
 */
class AdapterOperationMode
{
public:
  AdapterOperationMode(ros::NodeHandle& nh);
  virtual ~AdapterOperationMode() = default;

protected:
  /**
   * @brief Stores the operation mode and initializes the operation mode
   * service, the first time the function is called.
   */
  void updateOperationMode(const int8_t new_op_mode);

private:
  /**
   * @brief Initializes the operation mode service.
   */
  void initOperationModeService();

  bool getOperationMode(GetOperationMode::Request& req,
                        GetOperationMode::Response& res);

private:
  //! Is the service advertising the operation mode initialized?
  bool service_initialized_ {false};

  //! Store the current operation mode according to OperationModes.msg
  int8_t op_mode_ {OperationModes::UNKNOWN};

  //! The node handle
  ros::NodeHandle& nh_;

  //! Server serving a service to ask whether a brake test is currently required
  ros::ServiceServer operation_mode_server_;

};

} // namespace prbt_hardware_support
#endif // ADAPTER_OPERATION_MODE_H
