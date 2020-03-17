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

#include <pilz_control/cartesian_speed_monitor.h>

namespace pilz_control
{

bool hasOnlyFixedParentJoints(const moveit::core::LinkModel * const &link)
{
  auto parent_link {link};
  while (parent_link != nullptr && parent_link->parentJointIsFixed())
  {
    parent_link = parent_link->getParentLinkModel();
  }
  return parent_link == nullptr;
}

CartesianSpeedMonitor::CartesianSpeedMonitor(const std::vector<std::string> &joint_names,
                                             const robot_model::RobotModelConstPtr &kinematic_model)
  : joint_names_(joint_names)
  , kinematic_model_(kinematic_model)
{
  assert(joint_names_.size() == kinematic_model_->getVariableCount());
}

void CartesianSpeedMonitor::init()
{
  std::vector<std::string> frames_to_observe;
  auto links = kinematic_model_->getLinkModels();

  for (const auto& link : links)
  {
    if(!hasOnlyFixedParentJoints(link))
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
                                                       const double& time_delta,
                                                       const double& speed_limit)
{
  if (speed_limit < 0.0)
  {
    return true;
  }

  state_current_->setVariablePositions(joint_names_, current_position);
  state_desired_->setVariablePositions(joint_names_, desired_position);

  state_current_->updateLinkTransforms();
  state_desired_->updateLinkTransforms();

  for(const auto & link : monitored_links_)
  {
    auto speed = linkSpeed(state_current_, state_desired_, link, time_delta);

    if(speed > speed_limit)
    {
      ROS_ERROR_STREAM("Speed limit violated by link '" << link->getName() << "'! Desired Speed: " << speed
                       << ", speed_limit: " << speed_limit);
      return false;
    }
  }

  return true;
}

double CartesianSpeedMonitor::linkSpeed(const robot_state::RobotStateConstPtr& current_state, // Important! Keep this RobotStateConstPtr to allow efficient
                                        const robot_state::RobotStateConstPtr& desired_state, // getGlobalLinkTransform calls
                                        const moveit::core::LinkModel* link,
                                        const double& time_delta)
{
  auto p1_cart {current_state->getGlobalLinkTransform(link)};
  auto p2_cart {desired_state->getGlobalLinkTransform(link)};

  auto dist {(p2_cart.translation() - p1_cart.translation()).norm()};
  auto speed {dist / time_delta};

  return speed;
}


}  // namespace pilz_control
