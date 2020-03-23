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

//! @brief Monitors the cartesian speed of all moving links of a position-controlled robot.
class CartesianSpeedMonitor
{
public:
  /**
   * @brief Constructor.
   *
   * @param joint_names List of joint names of controlled joints. The same order has to be used for joint positions.
   * @param kinematic_model MoveIt robot model used for computing the forward kinematics.
   * @throw CartesianSpeedMonitorException if the number of joint names does not match the variable count in kinematic_model.
   */
  CartesianSpeedMonitor(const std::vector<std::string> &joint_names,
                        const robot_model::RobotModelConstPtr &kinematic_model);

   //! @brief Initialize speed monitor by collecting all moveable links.
  void init();

  /**
   * @brief Check if cartesian speed of all links is below the speed limit.
   *
   * @param current_position Current positions of controlled joints.
   * @param desired_position Desired positions of controlled joints.
   * @param time_delta Time for reaching the desired positions.
   * @param speed_limit
   *
   * @returns False if the speed limit is violated, otherwise true.
   */
  bool cartesianSpeedIsBelowLimit(const std::vector<double>& current_position,
                                  const std::vector<double>& desired_position,
                                  const double& time_delta,
                                  const double& speed_limit);

public:
  /**
   * @brief Compute the cartesian speed of a single robot link.
   *
   * @param current_state
   * @param desired_state
   * @param link
   * @param time_delta Time for reaching the desired state.
   *
   * @returns The computed cartesian speed.
   */
  static double linkSpeed(const robot_state::RobotStateConstPtr& current_state, // !!! Keep this RobotStateConstPtr for
                          const robot_state::RobotStateConstPtr& desired_state, // efficient getGlobalLinkTransform calls
                          const moveit::core::LinkModel* link,
                          const double& time_delta);

private:
  robot_model::RobotModelConstPtr kinematic_model_;
  //! @brief The robot states are kept in order to allow efficient getGlobalLinkTransform calls
  robot_state::RobotStatePtr state_current_;
  //! @brief The robot states are kept in order to allow efficient getGlobalLinkTransform calls
  robot_state::RobotStatePtr state_desired_;

  std::vector<std::string> joint_names_;

  //! @brief All moveable robot links are monitored
  std::vector< const robot_model::LinkModel * > monitored_links_;
};

}  // namespace pilz_control

#endif // CARTESIAN_SPEED_MONITOR_H
