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

#ifndef PILZ_CONTROL_PILZ_JOINT_TRAJECTORY_CONTROLLER_IMPL_H
#define PILZ_CONTROL_PILZ_JOINT_TRAJECTORY_CONTROLLER_IMPL_H

namespace pilz_joint_trajectory_controller
{

namespace ph = std::placeholders;

template <class SegmentImpl, class HardwareInterface>
PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
PilzJointTrajectoryController()
{
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::init(HardwareInterface* hw,
                                                                     ros::NodeHandle&   root_nh,
                                                                     ros::NodeHandle&   controller_nh)

{
  bool res = JointTrajectoryController::init(hw, root_nh, controller_nh);

  hold_position_service = controller_nh.advertiseService("hold",
                                                         &PilzJointTrajectoryController::handleHoldRequest,
                                                         this);


  unhold_position_service = controller_nh.advertiseService("unhold",
                                                         &PilzJointTrajectoryController::handleUnHoldRequest,
                                                         this);

  is_executing_service_ = controller_nh.advertiseService("is_executing",
                                                         &PilzJointTrajectoryController::handleIsExecutingRequest,
                                                         this);

  return res;
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::is_executing()
{
  if (JointTrajectoryController::state_ != JointTrajectoryController::RUNNING)
  {
    return false;
  }

  // Get currently followed trajectory
  TrajectoryPtr curr_traj_ptr;
  JointTrajectoryController::curr_trajectory_box_.get(curr_traj_ptr);
  if (!curr_traj_ptr)
  {
    return false;
  }

  Trajectory& curr_traj = *curr_traj_ptr;

  bool is_executing {false};

  for (unsigned int i = 0; i < JointTrajectoryController::joints_.size(); ++i)
  {
    auto uptime {JointTrajectoryController::time_data_.readFromRT()->uptime.toSec()};
    typename TrajectoryPerJoint::const_iterator segment_it = findSegment(curr_traj[i], uptime);
    // Times that preceed the trajectory start time are ignored here, so is_executing() returns false
    // even if there is a current trajectory that will be executed in the future.
    if (segment_it != curr_traj[i].end() && uptime <= segment_it->endTime())
    {
      is_executing = true;
      break;
    }
  }

  return is_executing;
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
handleHoldRequest(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& response)
{
  std::lock_guard<std::mutex> lock(sync_mutex_);

  if(active_mode_ == Mode::HOLD)
  {
    response.message = "Already in hold mode";
    response.success = true;
    return true;
  }

  active_mode_ = Mode::HOLD;

  JointTrajectoryController::preemptActiveGoal();
  triggerMovementToHoldPosition();

  ros::Duration(JointTrajectoryController::stop_trajectory_duration_).sleep();

  response.message = "Holding mode enabled";
  response.success = true;
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
handleUnHoldRequest(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& response)
{
  std::lock_guard<std::mutex> lock(sync_mutex_);

  if(active_mode_ == Mode::UNHOLD)
  {
    response.message = "Already in unhold mode";
    response.success = true;
    return true;
  }

  active_mode_ = Mode::UNHOLD;

  response.message = "Default mode enabled";
  response.success = true;
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
handleIsExecutingRequest(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& response)
{
  response.success = is_executing();
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
updateTrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh, std::string* error_string)
{
  std::lock_guard<std::mutex> lock(sync_mutex_);
  if(active_mode_ == Mode::HOLD)
  {
    return updateStrategyWhileHolding(msg, gh, error_string);
  }

  // The default case
  return updateStrategyDefault(msg, gh, error_string);
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
updateStrategyDefault(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh, std::string* error_string)
{
  return JointTrajectoryController::updateTrajectoryCommand(msg, gh, error_string);
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
updateStrategyWhileHolding(const JointTrajectoryConstPtr&, RealtimeGoalHandlePtr, std::string* error_string)
{
  ROS_WARN_THROTTLE_NAMED(10, this->name_,
                          "Controller received new commands but won't react because it is currently in holding mode.");
  return false;
}

template <class SegmentImpl, class HardwareInterface>
void PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
triggerMovementToHoldPosition()
{
  JointTrajectoryController::setHoldPosition(this->time_data_.readFromRT()->uptime);
}


}  // namespace pilz_joint_trajectory_controller

#endif  // PILZ_CONTROL_PILZ_JOINT_TRAJECTORY_CONTROLLER_IMPL_H
