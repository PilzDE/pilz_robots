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

#ifndef CARTESIAN_SPEED_MONITOR_H
#define CARTESIAN_SPEED_MONITOR_H

#include <vector>
#include <string>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model.h>

namespace pilz_control
{
/**
 * @brief Compute the cartesian speed of a single robot link.
 *
 * @param current_state Current state of the robot. The link transforms have to be up-to-date.
 * @param desired_state Desired state of the robot in the future. The link transforms have to be up-to-date.
 * @param link Robot link under observation.
 * @param time_delta Time[s] for reaching the desired state.
 * @returns Cartesian speed[m/s].
 */
static double linkSpeed(const robot_state::RobotState* current_state,  // We require that the link transforms
                        const robot_state::RobotState* desired_state,  // are up-to-date in this context, such
                                                                       // that we can use the more efficient
                                                                       // const-version getGlobalLinkTransform()
                        const moveit::core::LinkModel* link, const double& time_delta)
{
  const auto p1_cart{ current_state->getGlobalLinkTransform(link) };
  const auto p2_cart{ desired_state->getGlobalLinkTransform(link) };

  const auto dist{ (p2_cart.translation() - p1_cart.translation()).norm() };
  const auto speed{ dist / time_delta };

  return speed;
}

//! @brief Monitors the cartesian speed of all links of a position-controlled robot (end effector is excluded).
class CartesianSpeedMonitor
{
public:
  /**
   * @param joint_names List of joint names of controlled joints. The same order has to be used for joint positions.
   * @param kinematic_model MoveIt robot model used for computing the forward kinematics.
   * @throw RobotModelVariableNamesMismatch if the joint names do not match variable names in kinematic_model.
   */
  CartesianSpeedMonitor(const std::vector<std::string>& joint_names,
                        const robot_model::RobotModelConstPtr& kinematic_model);

  //! @brief Prepares the CartesianSpeedMonitor for execution.
  void init();

  /**
   * @brief Check if cartesian speed of all monitored links is below the speed limit.
   *
   * @param current_position Current positions[rad] of controlled joints in the order of \ref joint_names_.
   * @param desired_position Desired positions[rad] of controlled joints in the order of \ref joint_names_.
   * @param time_delta Time[s] for reaching the desired positions.
   * @param speed_limit Speed limit in m/s.
   *
   * @returns False if the speed limit is violated, otherwise true.
   */
  bool cartesianSpeedIsBelowLimit(const std::vector<double>& current_position,
                                  const std::vector<double>& desired_position, const double& time_delta,
                                  const double& speed_limit);

private:
  const robot_model::RobotModelConstPtr kinematic_model_;
  //! @brief The robot states are kept in order to allow efficient getGlobalLinkTransform calls.
  robot_state::RobotStatePtr state_current_;
  //! @brief The robot states are kept in order to allow efficient getGlobalLinkTransform calls.
  robot_state::RobotStatePtr state_desired_;

  const std::vector<std::string> joint_names_;

  //! @brief Stores all monitored links.
  std::vector<const robot_model::LinkModel*> monitored_links_;
};

}  // namespace pilz_control

#endif  // CARTESIAN_SPEED_MONITOR_H
