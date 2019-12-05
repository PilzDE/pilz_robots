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

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <pilz_control/pilz_cartesian_speed_observing_controller.h>

namespace pilz_cartesian_speed_observing_controller
{
  bool PilzCartesianSpeedObservingController::init(hardware_interface::JointStateInterface* hw,
                                  ros::NodeHandle&                         root_nh,
                                  ros::NodeHandle&                         controller_nh)
  {
    const std::vector<std::string>& joint_names = hw->getNames();
    num_hw_joints_ = joint_names.size();
    for (size_t i=0; i<num_hw_joints_; i++)
    {
      ROS_DEBUG("Got joint %s", joint_names[i].c_str());
    }

    ROS_ERROR("Successfully initialized PilzCartesianSpeedObservingController!");

    return true;
    
  }

  void PilzCartesianSpeedObservingController::starting(const ros::Time& time)
  {
  }

  void PilzCartesianSpeedObservingController::update(const ros::Time& time, const ros::Duration& /*period*/)
  {
    // TODO
  }


  void PilzCartesianSpeedObservingController::stopping(const ros::Time& /*time*/)
  {}

}  // namespace pilz_cartesian_speed_observing_controller

PLUGINLIB_EXPORT_CLASS(pilz_cartesian_speed_observing_controller::PilzCartesianSpeedObservingController, controller_interface::ControllerBase)
