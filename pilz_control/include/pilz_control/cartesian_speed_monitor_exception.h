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

#ifndef PILZ_CONTROL_CARTESIAN_SPEED_MONITOR_EXCEPTION_H
#define PILZ_CONTROL_CARTESIAN_SPEED_MONITOR_EXCEPTION_H

#include <stdexcept>
#include <string>

namespace pilz_control
{
/**
 * @brief Throw this exception when given names do not match the robot model variable names.
 */
class RobotModelVariableNamesMismatch : public std::invalid_argument
{
public:
  RobotModelVariableNamesMismatch()
    : std::invalid_argument("The given names do not match the robot model variable names.")
  {
  }
};

}  // namespace pilz_control

#endif  // PILZ_CONTROL_CARTESIAN_SPEED_MONITOR_EXCEPTION_H
