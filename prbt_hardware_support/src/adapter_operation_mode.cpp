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

#include <prbt_hardware_support/adapter_operation_mode.h>

namespace prbt_hardware_support
{

static const std::string SERVICE_NAME_GET_OPERATION_MODE = "/prbt/get_operation_mode";

AdapterOperationMode::AdapterOperationMode(ros::NodeHandle& nh)
  : nh_(nh)
{
  op_mode_.time_stamp = ros::Time::now();
  op_mode_.value = OperationModes::UNKNOWN;

  operation_mode_server_ = nh_.advertiseService(SERVICE_NAME_GET_OPERATION_MODE,
                                                &AdapterOperationMode::getOperationMode,
                                                this);
}

void AdapterOperationMode::updateOperationMode(const OperationModes& new_op_mode)
{
  const int8_t last_op_mode {op_mode_.value};
  op_mode_ = new_op_mode;
  if (op_mode_.value != last_op_mode)
  {
    ROS_INFO_STREAM( "Operation Mode switch: "
                     << static_cast<int>(last_op_mode)
                     << " -> "
                     << static_cast<int>(new_op_mode.value) );
  }
}

bool AdapterOperationMode::getOperationMode(GetOperationMode::Request& /*req*/,
                                            GetOperationMode::Response& res)
{
  res.mode = op_mode_;
  return true;
}

} // namespace prbt_hardware_support
