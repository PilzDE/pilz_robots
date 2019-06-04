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

#ifndef PRBT_HARDWARE_SUPPORT_GET_PARAM_H
#define PRBT_HARDWARE_SUPPORT_GET_PARAM_H

#include <string>
#include <stdexcept>
#include <sstream>

#include <ros/ros.h>

namespace prbt_hardware_support
{

/**
   * @brief Exception used by the getParam function.
   */
class GetParamException : public std::runtime_error
{
public:
  GetParamException(const std::string& msg);
};

inline GetParamException::GetParamException(const std::string& msg)
  : std::runtime_error (msg)
{

}

template<class T>
T getParam(const ros::NodeHandle& nh, const std::string& param_name)
{
  T ret_val;
  if ( !nh.getParam(param_name, ret_val) )
  {
    std::ostringstream os;
    os << "Parameter \"" << param_name << "\" not given";
    throw GetParamException(os.str());
  }
  return ret_val;
}

}

#endif // PRBT_HARDWARE_SUPPORT_GET_PARAM_H