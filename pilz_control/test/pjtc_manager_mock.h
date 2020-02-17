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

#ifndef PILZ_CONTROL_CONTROLLER_MANAGER_MOCK_H
#define PILZ_CONTROL_CONTROLLER_MANAGER_MOCK_H

#include <future>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <std_srvs/Trigger.h>

#include <hardware_interface/robot_hw.h>

#include <pilz_control/pilz_joint_trajectory_controller.h>

namespace pilz_joint_trajectory_controller_test
{
/**
 * @brief Manages a single PilzJointTrajectoryController (PJTC).
 *
 * Allows direct access to a PilzJointTrajectoryController object for testing.
 * @note Intended for usage with simulated ros::Time.
 */
template <class SegmentImpl, class HWInterface>
class PJTCManagerMock
{
public:
  using Controller = pilz_joint_trajectory_controller::PilzJointTrajectoryController<SegmentImpl, HWInterface>;

public:
  PJTCManagerMock(hardware_interface::RobotHW* hardware, const std::string& controller_ns);

  bool loadController();
  void startController();
  //! Perform controller update at current (simulated) time.
  void update();

  /**
   * @brief The period is the time span between the last update-time and the current time.
   * @note This only makes sense with simulated time.
   */
  ros::Duration getCurrentPeriod();

  bool triggerHold(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
  bool triggerUnHold(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

  std::future<bool> triggerHoldAsync(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

public:
  std::shared_ptr<Controller> controller_;

private:
  ros::NodeHandle nh_{ "~" };
  std::string controller_ns_;
  ros::Time last_update_time_;
  ros::Duration last_period_;
  hardware_interface::RobotHW* hardware_;
};

template <class SegmentImpl, class HWInterface>
PJTCManagerMock<SegmentImpl, HWInterface>::PJTCManagerMock(hardware_interface::RobotHW* hardware,
                                                           const std::string& controller_ns)
  : controller_ns_(controller_ns), hardware_(hardware)
{
}

template <class SegmentImpl, class HWInterface>
bool PJTCManagerMock<SegmentImpl, HWInterface>::loadController()
{
  ros::NodeHandle controller_nh{ controller_ns_ };
  controller_.reset(new Controller());
  if (controller_->init(hardware_->get<HWInterface>(), nh_, controller_nh))
  {
    controller_->state_ = controller_interface::ControllerBase::ControllerState::INITIALIZED;
    return true;
  }
  return false;
}

template <class SegmentImpl, class HWInterface>
void PJTCManagerMock<SegmentImpl, HWInterface>::startController()
{
  ros::Time current_time{ ros::Time::now() };
  controller_->starting(current_time);
  controller_->state_ = controller_interface::ControllerBase::ControllerState::RUNNING;
  last_update_time_ = current_time;
}

template <class SegmentImpl, class HWInterface>
void PJTCManagerMock<SegmentImpl, HWInterface>::update()
{
  ros::Time current_time{ ros::Time::now() };
  last_period_ = current_time - last_update_time_;
  last_update_time_ = current_time;
  controller_->update(last_update_time_, last_period_);
}

template <class SegmentImpl, class HWInterface>
ros::Duration PJTCManagerMock<SegmentImpl, HWInterface>::getCurrentPeriod()
{
  return last_period_;
}

template <class SegmentImpl, class HWInterface>
bool PJTCManagerMock<SegmentImpl, HWInterface>::triggerHold(std_srvs::TriggerRequest& request,
                                                            std_srvs::TriggerResponse& response)
{
  return controller_->handleHoldRequest(request, response);
}

template <class SegmentImpl, class HWInterface>
bool PJTCManagerMock<SegmentImpl, HWInterface>::triggerUnHold(std_srvs::TriggerRequest& request,
                                                              std_srvs::TriggerResponse& response)
{
  return controller_->handleUnHoldRequest(request, response);
}

template <class SegmentImpl, class HWInterface>
std::future<bool> PJTCManagerMock<SegmentImpl, HWInterface>::triggerHoldAsync(std_srvs::TriggerRequest& request,
                                                                              std_srvs::TriggerResponse& response)
{
  return std::async(std::launch::async,
                    [this, &request, &response]() { return controller_->handleHoldRequest(request, response); });
}

}  // namespace pilz_joint_trajectory_controller_test

#endif  // PILZ_CONTROL_CONTROLLER_MANAGER_MOCK_H
