/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PILZ_UTILS_GET_PARAM_H
#define PILZ_UTILS_GET_PARAM_H

#include <string>
#include <stdexcept>
#include <sstream>

#include <ros/ros.h>

namespace pilz_utils
{
/**
 * @brief Exception used by the getParam function.
 */
class GetParamException : public std::runtime_error
{
public:
  GetParamException(const std::string& msg);
};

inline GetParamException::GetParamException(const std::string& msg) : std::runtime_error(msg)
{
}

template <class T>
T getParam(const ros::NodeHandle& nh, const std::string& param_name)
{
  T ret_val;
  if (!nh.getParam(param_name, ret_val))
  {
    std::ostringstream os;
    os << "Parameter \"" << param_name << "\" not given";
    throw GetParamException(os.str());
  }
  return ret_val;
}

}  // namespace pilz_utils

#endif  // PILZ_UTILS_GET_PARAM_H
