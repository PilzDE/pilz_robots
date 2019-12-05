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

#ifndef CARTESIAN_SPEED_MONITOR_H
#define CARTESIAN_SPEED_MONITOR_H

#include <future>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

namespace pilz_control
{
class CartesianSpeedMonitor
{
  public:
    CartesianSpeedMonitor()
    {

    }

    void init()
    {   
      bool load_kinematics_solvers = true; // OTHERWISE warning TODO investigate what is best todo here. Check if loaded?
      robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description", load_kinematics_solvers));
      kinematic_model_ = robot_model_loader_->getModel();

      std::vector<std::string> frames_to_observe;
      auto links = kinematic_model_->getLinkModels();
      
      for (const auto& link : links)
      {
        if(!link->parentJointIsFixed()) // Not sure about this...
        {
          observed_links_.insert(observed_links_.begin(), link);
          ROS_ERROR_STREAM("Created observer for " << link->getName());
        }
      }

      state_current_.reset(new robot_state::RobotState(kinematic_model_));
      state_desired_.reset(new robot_state::RobotState(kinematic_model_));

    }

    // Note that this is optimized to hit on the first limit in order to be fast, do not "refactor/clean" this away
    bool cartesianSpeedIsBelowLimit(std::vector< double > current_position, std::vector< double > desired_position, const double time_delta, const double speed_limit)
    {
      state_current_->setVariablePositions(current_position);
      state_desired_->setVariablePositions(desired_position);

      state_current_->updateLinkTransforms();
      state_desired_->updateLinkTransforms();

      for(const auto & link : observed_links_)
      {
        auto speed = linkSpeed(state_current_, state_desired_, link, time_delta);

        if(speed > speed_limit)
        {
          std::cerr << "Speed limit violated by link" << link->getName() << "! Desired Speed: " << speed << " speed_limit: " << speed_limit << std::endl;
          return false;
        }
      }

      return true;
    }

    static double linkSpeed(const robot_state::RobotStateConstPtr& current_state, // Important! Keep this RobotStateConstPtr to allow efficient
                            const robot_state::RobotStateConstPtr& desired_state, // getGlobalLinkTransform calls
                            const moveit::core::LinkModel* link,
                            const double time_delta)
    {
      auto p1_cart = current_state->getGlobalLinkTransform(link);
      auto p2_cart = desired_state->getGlobalLinkTransform(link);

      auto dist = (p2_cart.translation() - p1_cart.translation()).norm();

      auto speed = dist / time_delta;
     
      return speed;
    }

  private:

    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    robot_model::RobotModelPtr kinematic_model_;
    robot_state::RobotStatePtr state_current_;
    robot_state::RobotStatePtr state_desired_;

    std::vector< const robot_model::LinkModel * > observed_links_;
};

}  // namespace pilz_control

#endif  //  SPEED_CALCULATOR_MANAGER_H

  
  
  
