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

#ifndef PILZ_CONTROL_ROBOT_DRIVER_MOCK_H
#define PILZ_CONTROL_ROBOT_DRIVER_MOCK_H

#include <memory>
#include <string>

#include "pjtc_manager_mock.h"
#include "robot_mock.h"

namespace pilz_joint_trajectory_controller_test
{
/**
 * @brief Connects controller to robot mock via controller manager.
 */
template <class ControllerManager>
class RobotDriverMock
{
private:
  using ManagerPtr = std::shared_ptr<ControllerManager>;

public:
  RobotDriverMock(const std::string& controller_ns);

  //! @brief Sync robot and controller.
  void update();

  ManagerPtr getManager() const;

  bool isRobotMoving() const;

  std::vector<double> getJointPositions() const;

private:
  RobotMock robot_;
  ManagerPtr manager_;
};

template <class ControllerManager>
RobotDriverMock<ControllerManager>::RobotDriverMock(const std::string& controller_ns)
{
  manager_.reset(new ControllerManager(&robot_, controller_ns));
}

template <class ControllerManager>
void RobotDriverMock<ControllerManager>::update()
{
  robot_.read();
  manager_->update();
  robot_.write(manager_->getCurrentPeriod());
}

template <class ControllerManager>
typename RobotDriverMock<ControllerManager>::ManagerPtr RobotDriverMock<ControllerManager>::getManager() const
{
  return manager_;
}

template <class ControllerManager>
bool RobotDriverMock<ControllerManager>::isRobotMoving() const
{
  return robot_.isMoving();
}

template <class ControllerManager>
std::vector<double> RobotDriverMock<ControllerManager>::getJointPositions() const
{
  const auto joint_data = robot_.read();
  std::vector<double> positions;
  std::transform(joint_data.begin(), joint_data.end(), std::back_inserter(positions),
                 [](const JointData& data) { return data.pos; });
  return positions;
}

}  // namespace pilz_joint_trajectory_controller_test

#endif  // PILZ_CONTROL_ROBOT_DRIVER_MOCK_H
