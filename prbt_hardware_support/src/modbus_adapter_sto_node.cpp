/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#include <functional>

#include <ros/ros.h>

#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/modbus_adapter_sto.h>
#include <prbt_hardware_support/filter_pipeline.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/wait_for_service.h>
#include <std_srvs/SetBool.h>

using namespace prbt_hardware_support;

static const std::string STO_SERVICE_NAME{"safe_torque_off"};

// LCOV_EXCL_START
static void sendStoUpdate(ros::ServiceClient& sto_service, const bool sto)
{
  std_srvs::SetBool srv;
  srv.request.data = sto;
  if (!sto_service.call(srv))
  {
    ROS_ERROR_STREAM("STO service call failed");
  }

  if (!srv.response.success)
  {
    ROS_ERROR_STREAM(srv.response.message);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "modbus_adapter_sto");
  ros::NodeHandle nh;

  using std::placeholders::_1;

  ModbusApiSpec api_spec{nh};
  waitForService(STO_SERVICE_NAME);
  ros::ServiceClient sto_service = nh.serviceClient<std_srvs::SetBool>(STO_SERVICE_NAME);
  ModbusAdapterSto adapter_sto(std::bind(sendStoUpdate, sto_service, _1), api_spec);
  FilterPipeline filter_pipeline( nh, std::bind(&ModbusAdapterSto::modbusMsgCallback, &adapter_sto, _1) );

  ros::spin();

  return EXIT_FAILURE;
}
// LCOV_EXCL_STOP
