/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
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

#ifndef ROS_NOT_OK_EXCEPTION_H
#define ROS_NOT_OK_EXCEPTION_H

#include <stdexcept>
#include <string>

namespace pilz_testutils
{
// @brief Exception to be thrown if ros::ok() is expected to but true, but it's not
class ROSNotOkException : public std::runtime_error
{
public:
  ROSNotOkException(const std::string& what_arg) : std::runtime_error(what_arg)
  {
  }
};

}  // namespace pilz_testutils

#endif  // ROS_NOT_OK_EXCEPTION_H