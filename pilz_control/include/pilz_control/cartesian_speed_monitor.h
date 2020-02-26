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

class CartesianSpeedMonitor
{
public:
  CartesianSpeedMonitor(const std::vector<std::string> &joint_names,
                        const robot_model::RobotModelConstPtr &kinematic_model);

  void init();

  bool cartesianSpeedIsBelowLimit(const std::vector<double>& current_position,
                                  const std::vector<double>& desired_position,
                                  const double& time_delta,
                                  const double& speed_limit);

public:
  static double linkSpeed(const robot_state::RobotStateConstPtr& current_state, // Important! Keep this RobotStateConstPtr to allow efficient
                          const robot_state::RobotStateConstPtr& desired_state, // getGlobalLinkTransform calls
                          const moveit::core::LinkModel* link,
                          const double& time_delta);

private:
  robot_model::RobotModelConstPtr kinematic_model_;
  robot_state::RobotStatePtr state_current_;
  robot_state::RobotStatePtr state_desired_;

  std::vector<std::string> joint_names_;

  std::vector< const robot_model::LinkModel * > observed_links_;
};

}  // namespace pilz_control

#endif // CARTESIAN_SPEED_MONITOR_H
