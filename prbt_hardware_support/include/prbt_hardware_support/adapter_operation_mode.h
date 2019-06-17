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
  void init();
  void updateOperationMode(int8_t operation_mode);
  bool getOperationMode(GetOperationMode::Request&, GetOperationMode::Response& response);

private:
  //! Is the node initialized?
  bool initialized_;

  //! Store the current operation mode according to OperationModes.msg
  int8_t current_operation_mode_;

  //! The node handle
  ros::NodeHandle& nh_;

  //! Server serving a service to ask whether a brake test is currently required
  ros::ServiceServer operation_mode_server_;

};

} // namespace prbt_hardware_support
#endif // ADAPTER_OPERATION_MODE_H
