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

#include <algorithm>

#include <pilz_control/cartesian_speed_monitor.h>
#include <pilz_control/cartesian_speed_monitor_exception.h>

namespace pilz_control
{
//! @return True if all parent joints in the chain below the specified link are fixed, otherwise false.
bool hasOnlyFixedParentJoints(const moveit::core::LinkModel* const& link)
{
  auto parent_link{ link };
  while (parent_link != nullptr && parent_link->parentJointIsFixed())
  {
    parent_link = parent_link->getParentLinkModel();
  }
  return parent_link == nullptr;
}

//! @return True if the link is part of an end effector, otherwise false.
bool isEndEffectorLink(const moveit::core::LinkModel* const& link, const robot_model::RobotModelConstPtr& robot_model)
{
  const auto& end_effectors{ robot_model->getEndEffectors() };
  for (const auto& end_effector : end_effectors)
  {
    const auto& eef_parent_group_and_link_name{ end_effector->getEndEffectorParentGroup() };
    auto parent_link{ link->getParentLinkModel() };
    while (parent_link != nullptr)
    {
      if (parent_link->getName() == eef_parent_group_and_link_name.second)
      {
        return true;
      }
      parent_link = parent_link->getParentLinkModel();
    }
  }
  return false;
}

CartesianSpeedMonitor::CartesianSpeedMonitor(const std::vector<std::string>& joint_names,
                                             const robot_model::RobotModelConstPtr& kinematic_model)
  : joint_names_(joint_names), kinematic_model_(kinematic_model)
{
  const auto& variable_names = kinematic_model_->getVariableNames();
  if (std::any_of(joint_names_.begin(), joint_names_.end(), [variable_names](const std::string& name) {
        return std::find(variable_names.begin(), variable_names.end(), name) == variable_names.end();
      }))
  {
    throw RobotModelVariableNamesMismatch();
  }
}

void CartesianSpeedMonitor::init()
{
  const auto& links = kinematic_model_->getLinkModels();

  for (const auto& link : links)
  {
    if (!hasOnlyFixedParentJoints(link) && !isEndEffectorLink(link, kinematic_model_))
    {
      monitored_links_.push_back(link);
      ROS_INFO_STREAM("Monitoring cartesian speed of link " << link->getName());
    }
  }

  state_current_.reset(new robot_state::RobotState(kinematic_model_));
  state_desired_.reset(new robot_state::RobotState(kinematic_model_));
}

bool CartesianSpeedMonitor::cartesianSpeedIsBelowLimit(const std::vector<double>& current_position,
                                                       const std::vector<double>& desired_position,
                                                       const double& time_delta, const double& speed_limit)
{
  if (speed_limit < 0.0)
  {
    return true;
  }

  state_current_->setVariablePositions(joint_names_, current_position);
  state_desired_->setVariablePositions(joint_names_, desired_position);

  state_current_->updateLinkTransforms();
  state_desired_->updateLinkTransforms();

  for (const auto& link : monitored_links_)
  {
    const auto& speed = linkSpeed(state_current_.get(), state_desired_.get(), link, time_delta);

    if (speed > speed_limit)
    {
      ROS_ERROR_STREAM("Speed limit violated by link '" << link->getName() << "'! Desired Speed: " << speed
                                                        << "m/s, speed_limit: " << speed_limit << "m/s");
      return false;
    }
  }

  return true;
}

}  // namespace pilz_control
