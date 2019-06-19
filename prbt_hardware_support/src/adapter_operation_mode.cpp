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
  : initialized_(false)
  , nh_(nh) {
}

void AdapterOperationMode::init()
{
  operation_mode_server_ = nh_.advertiseService(SERVICE_NAME_GET_OPERATION_MODE,
                                                &AdapterOperationMode::getOperationMode,
                                                this);
}

void AdapterOperationMode::updateOperationMode(int8_t mode)
{
  ROS_INFO_STREAM("Mode switched: " << (int)current_operation_mode_ << " -> " << (int)mode);
  current_operation_mode_ = mode;

  // when the first data is received, the node is initialized (i.e. the service advertised)
  // "lazy initialization"
  if(!initialized_) {
    init();
    initialized_ = true;
  }
}

bool AdapterOperationMode::getOperationMode(GetOperationMode::Request&,
                                            GetOperationMode::Response& res)
{
  res.mode.value = current_operation_mode_;
  return true;
}

}
