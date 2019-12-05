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

#include <chrono>
#include <future>
#include <string>

#include <std_srvs/Trigger.h>

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <pilz_control/pilz_cartesian_speed_observing_controller.h>

#include <pilz_utils/wait_for_service.h>

static const std::string HOLD_SERVICE_NAME{"/prbt/manipulator_joint_trajectory_controller/hold"};

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
      ROS_ERROR("Got joint %s", joint_names[i].c_str());
    }

    for (size_t i=0; i<num_hw_joints_; i++){
      joint_state_.push_back(hw->getHandle(joint_names[i]));
    }

    // pilz_utils::waitForService(HOLD_SERVICE_NAME);
    // hold_client_ = root_nh.serviceClient<std_srvs::Trigger>(HOLD_SERVICE_NAME);

    ROS_ERROR("Successfully initialized PilzCartesianSpeedObservingController!");

    cartesian_speed_monitor.reset(new pilz_control::CartesianSpeedMonitor());
    cartesian_speed_monitor->init();

    return true;
    
  }

  void PilzCartesianSpeedObservingController::starting(const ros::Time& time)
  {
  }

  void PilzCartesianSpeedObservingController::update(const ros::Time& time, const ros::Duration& period)
  {
    JointPositions current_positions;
    for (size_t i=0; i<num_hw_joints_; i++){
      current_positions.push_back(joint_state_[i].getPosition());

      // TODO Recheck what is set on the real system
      // ROS_ERROR_STREAM("Pos " << joint_state_[i].getPosition());
      // ROS_ERROR_STREAM("Vel " << joint_state_[i].getVelocity());
      // ROS_ERROR_STREAM("Eff " << joint_state_[i].getEffort());
    }

    if(!first_run)
    {

    // if (is_hold_running_)
    // {
    //   if (hold_success_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    //   {
    //     is_hold_running_ = false;
    //     if (hold_success_.get())
    //     {
    //       ROS_INFO("Hold call was successful.");
    //     }
    //     else
    //     {
    //       ROS_ERROR("Hold call was not successful.");
    //     }
    //   }
    // }

    if(!cartesian_speed_monitor->cartesianSpeedIsBelowLimit(
                                          last_positions,
                                          current_positions, 
                                          period.toSec(), 
                                          0.25 /*limit */))
    {
      ROS_ERROR("Above limit.");
      // if (!is_hold_running_)
      // {
      //   ROS_INFO("Trigger hold.");
      //   triggerHold();
      //   is_hold_running_ = true;
      // }
      // else
      // {
      //   ROS_INFO("Hold still running.");
      // }
    }
    
    }
    last_positions = current_positions;
    first_run = false;
    
  }


  void PilzCartesianSpeedObservingController::stopping(const ros::Time& /*time*/)
  {}

  void PilzCartesianSpeedObservingController::triggerHold()
  {
    hold_success_ = std::async(std::launch::async, [this]{
      std_srvs::Trigger srv;
      if (!hold_client_.call(srv))
      {
        ROS_ERROR("Failed to call service %s.", hold_client_.getService().c_str());
        return false;
      }
      if (!srv.response.success)
      {
        ROS_ERROR("Service response: %s", srv.response.message.c_str());
        return false;
      }
      return true;
    });
  }

}  // namespace pilz_cartesian_speed_observing_controller

PLUGINLIB_EXPORT_CLASS(pilz_cartesian_speed_observing_controller::PilzCartesianSpeedObservingController, controller_interface::ControllerBase)
