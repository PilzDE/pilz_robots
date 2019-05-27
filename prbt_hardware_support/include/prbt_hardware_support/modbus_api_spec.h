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

#ifndef MODBUS_API_SPEC_H
#define MODBUS_API_SPEC_H

#include <ros/ros.h>

namespace prbt_hardware_support
{

/**
 * @brief Expection thrown by prbt_hardware_support::PilzModbusReadClient
 */
class ModbusApiSpecException : public std::runtime_error
{
public:
  ModbusApiSpecException( const std::string& what_arg ):
    std::runtime_error(what_arg)
  {
  }
};

/**
 * @brief Specifies the meaning of the holding registers.
 *
 * Currently specifies in which registers version and braketest_request are defined.
 */
class ModbusApiSpec
{
public:

    ModbusApiSpec(unsigned int version_register,
                  unsigned int braketest_register,
                  unsigned int operation_mode_register):
    version_register_(version_register),
    braketest_register_(braketest_register),
    operation_mode_register_(operation_mode_register)
    {

    };


  ModbusApiSpec(ros::NodeHandle &nh)
  {
    static const std::string PARAM_API_SPEC_VERSION_MODBUS{"api_spec/VERSION"};
    static const std::string PARAM_API_SPEC_BRAKETEST_REQUEST{"api_spec/BRAKETEST_REQUEST"};
    static const std::string PARAM_API_SPEC_OPERATION_MODE{"api_spec/OPERATION_MODE"};

    // LCOV_EXCL_START Simple parameter reading not analyzed
    int version_register{0};
    if (!nh.getParam(PARAM_API_SPEC_VERSION_MODBUS, version_register))
    {
      ROS_ERROR("No version register given in api spec");
      throw ModbusApiSpecException("No version register given in api spec");
    }

    int operation_mode_register{0};
    if (!nh.getParam(PARAM_API_SPEC_OPERATION_MODE, operation_mode_register))
    {
      ROS_ERROR("No braketest register given in api spec");
      throw ModbusApiSpecException("No braketest register given in api spec");
    }

  }

  unsigned int version_register_;
  unsigned int braketest_register_;
  unsigned int operation_mode_register_;
};

} // namespace prbt_hardware_support
#endif // MODBUS_API_SPEC_H
