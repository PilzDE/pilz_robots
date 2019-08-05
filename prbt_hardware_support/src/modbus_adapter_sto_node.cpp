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

#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_VECTOR_SIZE 30 //or whatever you need
#define BOOST_MPL_LIMIT_MAP_SIZE 30 //or whatever you need

#include <ros/ros.h>

#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/modbus_adapter_sto.h>

using namespace prbt_hardware_support;

// LCOV_EXCL_START
int main(int argc, char **argv)
{
  ros::init(argc, argv, "modbus_adapter_sto");
  ros::NodeHandle nh;

  ModbusApiSpec api_spec{nh};

  ModbusAdapterSto adapter_sto(nh, api_spec);

  ros::spin();

  return EXIT_FAILURE;
}
// LCOV_EXCL_STOP
