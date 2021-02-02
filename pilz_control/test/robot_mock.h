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

#ifndef PILZ_CONTROL_ROBOT_MOCK_H
#define PILZ_CONTROL_ROBOT_MOCK_H

#include <array>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

constexpr unsigned int NUM_JOINTS{ 2 };
constexpr std::array<const char*, NUM_JOINTS> JOINT_NAMES = { "shoulder_to_right_arm", "shoulder_to_left_arm" };
constexpr double JOINT_VELOCITY_EPS{ 0.0001 };

struct JointData
{
  double pos{ 0.0 };
  double vel{ 0.0 };
  double eff{ 0.0 };
  double cmd{ 0.0 };
};

/**
 * @brief The RobotMock used by the unit- and integrationtest of the pilz_joint_trajectory_controller.
 * Registers NUM_JOINTS JointStateHandles with the interface to allow interaction with the controller_manager.
 */
class RobotMock : public hardware_interface::RobotHW
{
public:
  RobotMock();

  std::array<JointData, NUM_JOINTS> read() const;
  void write(const ros::Duration& period);

  bool isMoving(const double& eps = JOINT_VELOCITY_EPS) const;

private:
  std::array<JointData, NUM_JOINTS> data_{ JointData(), JointData() };
  hardware_interface::PositionJointInterface pos_jnt_interface_;
  hardware_interface::JointStateInterface jnt_state_interface_;
};

#endif  // PILZ_CONTROL_ROBOT_MOCK_H
