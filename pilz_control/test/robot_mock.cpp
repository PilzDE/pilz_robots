/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#include <sstream>

#include "robot_mock.h"

RobotMock::RobotMock()
{
  for (unsigned int i = 0; i < NUM_JOINTS; ++i)
  {
    std::ostringstream os;
    os << JOINT_NAMES.at(i);
    hardware_interface::JointStateHandle jnt_state_handle{ os.str(), &(data_.at(i).pos), &(data_.at(i).vel),
                                                           &(data_.at(i).eff) };
    hardware_interface::JointHandle jnt_handle{ jnt_state_handle, &(data_.at(i).cmd) };
    pos_jnt_interface_.registerHandle(jnt_handle);
    jnt_state_interface_.registerHandle(jnt_handle);
  }

  registerInterface(&pos_jnt_interface_);
  registerInterface(&jnt_state_interface_);
}

std::array<JointData, NUM_JOINTS> RobotMock::read() const
{
  return data_;
}

void RobotMock::write(const ros::Duration& period)
{
  for (unsigned int i = 0; i < NUM_JOINTS; ++i)
  {
    data_.at(i).vel = (data_.at(i).cmd - data_.at(i).pos) / period.toSec();
    data_.at(i).pos = data_.at(i).cmd;
  }
}

bool RobotMock::isMoving(const double& eps) const
{
  return std::any_of(data_.begin(), data_.end(), [eps](const JointData& data) { return std::abs(data.vel) > eps; });
}
