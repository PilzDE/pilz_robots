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

#include <ros/ros.h>

#include <prbt_hardware_support/modbus_adapter_brake_test.h>
#include <prbt_hardware_support/modbus_api_spec.h>

static const std::string PARAM_API_SPEC_VERSION_MODBUS{"api_spec/VERSION"};
static const std::string PARAM_API_SPEC_BRAKETEST_REQUEST{"api_spec/BRAKETEST_REQUEST"};

/**
 * @brief Starts a modbus brake test announcer and runs it until a failure occurs.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "modbus_adapter_brake_test");
  ros::NodeHandle nh{"~"};

  // LCOV_EXCL_START Simple parameter reading not analyzed
  int version_register{0};
  if (!nh.getParam(PARAM_API_SPEC_VERSION_MODBUS, version_register))
  {
    ROS_ERROR("No version register given in api spec");
    return EXIT_FAILURE;
  }

  int braketest_request_register{0};
  if (!nh.getParam(PARAM_API_SPEC_BRAKETEST_REQUEST, braketest_request_register))
  {
    ROS_ERROR("No braketest register given in api spec");
    return EXIT_FAILURE;
  }
  // LCOV_EXCL_STOP

  prbt_hardware_support::ModbusApiSpec api_spec(static_cast<unsigned int>(version_register),
                                                static_cast<unsigned int>(braketest_request_register));

  prbt_hardware_support::ModbusAdapterBrakeTest adapter_brake_test(nh, api_spec);

  ros::spin();

  return EXIT_FAILURE;
}
