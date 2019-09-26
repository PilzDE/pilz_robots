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

#ifndef OPERATION_MODE_SETUP_EXECUTOR_NODE_SERVICE_CALLS_H
#define OPERATION_MODE_SETUP_EXECUTOR_NODE_SERVICE_CALLS_H

#include <ros/ros.h>

#include <prbt_hardware_support/GetOperationMode.h>
#include <prbt_hardware_support/SetSpeedLimit.h>

namespace prbt_hardware_support
{

template<class T>
static bool setSpeedLimitSrv(T& srv_client, const double& speed_limit)
{
  SetSpeedLimit srv_msg;
  srv_msg.request.speed_limit = speed_limit;
  bool call_success = srv_client.call(srv_msg);
  if (!call_success)
  {
    ROS_ERROR_STREAM("No success calling service: " << srv_client.getService());
  }
  return call_success;
}

} // namespace prbt_hardware_support

#endif // OPERATION_MODE_SETUP_EXECUTOR_NODE_SERVICE_CALLS_H
