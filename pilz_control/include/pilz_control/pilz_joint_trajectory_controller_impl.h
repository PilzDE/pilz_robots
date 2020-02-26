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

#include <joint_trajectory_controller/joint_trajectory_segment.h>

namespace pilz_joint_trajectory_controller
{

namespace ph = std::placeholders;

template<class Segment>
bool isStopMotionFinished(const std::vector< TrajectoryPerJoint<Segment>>& traj,
                          const ros::Time& curr_uptime)
{
  for (unsigned int joint_index = 0; joint_index < traj.size(); ++joint_index)
  {
    assert(traj[joint_index].size() >= 1);
    const Segment& last_segment {traj[joint_index].back()};
    if (curr_uptime.toSec() < last_segment.endTime())
    {
      return false;
    }
  }
  // Whenever the time is up, we assume that the hold position is reached.
  return true;
};

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

  using robot_model_loader::RobotModelLoader;
  using pilz_control::CartesianSpeedMonitor;
  robot_model_loader_ = std::make_shared<RobotModelLoader>("robot_description", false);
  auto kinematic_model = robot_model_loader_->getModel();
  cartesian_speed_monitor_.reset(new CartesianSpeedMonitor(JointTrajectoryController::joint_names_, kinematic_model));
  cartesian_speed_monitor_->init();

  hold_position_service = controller_nh.advertiseService("hold",
                                                         &PilzJointTrajectoryController::handleHoldRequest,
                                                         this);


  unhold_position_service = controller_nh.advertiseService("unhold",
                                                           &PilzJointTrajectoryController::handleUnHoldRequest,
                                                           this);

  is_executing_service_ = controller_nh.advertiseService("is_executing",
                                                         &PilzJointTrajectoryController::handleIsExecutingRequest,
                                                         this);

  speed_limit_service_ = controller_nh.advertiseService("set_speed_limit",
                                                        &PilzJointTrajectoryController::handleSetSpeedLimitRequest,
                                                        this);

  stop_traj_builder_ = std::unique_ptr<joint_trajectory_controller::StopTrajectoryBuilder<SegmentImpl> >(new joint_trajectory_controller::StopTrajectoryBuilder<SegmentImpl>(JointTrajectoryController::getNumberOfJoints(),
                                                                                                                                                                             JointTrajectoryController::stop_trajectory_duration_,
                                                                                                                                                                             JointTrajectoryController::old_desired_state_));
  stop_traj_velocity_violation_ = JointTrajectoryController::createHoldTrajectory(JointTrajectoryController::getNumberOfJoints());

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
  ROS_ERROR_STREAM("Received hold request");
  TrajProcessingModeListener listener {TrajProcessingMode::hold};
  mode_->registerListener(&listener);
  if ( mode_->stoppingEvent() )
  {
    ROS_ERROR_STREAM("Switched to stopping mode");
    JointTrajectoryController::preemptActiveGoal();
    triggerMovementToHoldPosition();

    // Wait till stop motion finished by waiting for hold mode
    ROS_ERROR_STREAM("Wait for hold mode");
    listener.waitForMode();
    ROS_ERROR_STREAM("Switched to hold mode");

    response.message = "Holding mode enabled";
    response.success = true;
    return true;
  }

  ROS_ERROR_STREAM("Nothing todo for hold request");
  response.message = "Already in hold mode";
  response.success = true;
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
handleUnHoldRequest(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& response)
{
  ROS_ERROR_STREAM("Received unhold request");
  TrajProcessingModeListener listener {TrajProcessingMode::hold};
  mode_->registerListener(&listener);
  if ( !mode_->unholdEvent() )
  {
    ROS_ERROR_STREAM("Wait for hold mode");
    listener.waitForMode();
    if (mode_->unholdEvent())
    {
      ROS_ERROR_STREAM("Could not switch to unhold mode");
      response.message = "Could not switch to unhold mode (default mode)";
      response.success = false;
      return true;
    }
    ROS_ERROR_STREAM("Switched to unhold mode");
  }

  ROS_ERROR_STREAM("Unhold mode active");
  response.message = "Unhold mode (default mode) active";
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
  if (mode_->isholding())
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

template <class SegmentImpl, class HardwareInterface>
inline void PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
updateFuncExtensionPoint(const typename JointTrajectoryController::Trajectory& curr_traj,
                         const typename JointTrajectoryController::TimeData& time_data)
{
  switch(mode_->getCurrentMode())
  {
  case TrajProcessingMode::unhold:
  {
    if (!isPlannedCartesianVelocityOK(time_data.period))
    {
      mode_->stoppingEvent();
      stopMotion(time_data.uptime);
    }
    return;
  }
  case TrajProcessingMode::stopping:
  {
    ROS_ERROR_STREAM("Check if stop motion is finished");
    if ( isStopMotionFinished<typename JointTrajectoryController::Segment>(curr_traj, time_data.uptime) )
    {
      ROS_ERROR_STREAM("Stop motion is finished");
      mode_->stopMotionFinishedEvent();
    }
    return;
  }
  case TrajProcessingMode::hold:
    return;
  default:
    stopMotion(time_data.uptime);
    return;
  }
}

template <class SegmentImpl, class HardwareInterface>
inline bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
isPlannedCartesianVelocityOK(const ros::Duration& period) const
{
  return (cartesian_speed_monitor_->cartesianSpeedIsBelowLimit(JointTrajectoryController::old_desired_state_.position,
                                                               JointTrajectoryController::desired_state_.position,
                                                               period.toSec(),
                                                               cartesian_speed_limit_));
}

template <class SegmentImpl, class HardwareInterface>
void PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
stopMotion(const ros::Time& curr_uptime)
{
  triggerCancellingOfActiveGoal();

  stop_traj_builder_
      ->setStartTime(JointTrajectoryController::old_time_data_.uptime.toSec())
      ->buildTrajectory(stop_traj_velocity_violation_.get());
  stop_traj_builder_->reset();
  JointTrajectoryController::updateStates(curr_uptime, stop_traj_velocity_violation_.get());

  JointTrajectoryController::curr_trajectory_box_.set(stop_traj_velocity_violation_);
}

template <class SegmentImpl, class HardwareInterface>
inline void PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
triggerCancellingOfActiveGoal()
{
  RealtimeGoalHandlePtr active_goal(JointTrajectoryController::rt_active_goal_);
  if (!active_goal)
  {
    return;
  }
  JointTrajectoryController::rt_active_goal_.reset();
  active_goal->gh_.setCanceled();
  // TODO: Instead of the line above, I actually want to do this:
  //      active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
  //      active_goal->setCanceled(active_goal->preallocated_result_);
  // Unfortunately this does not work because sometimes the goal does not seems to get cancelled.
  // It has to be investigated why! -> Probably a threading problem.
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
handleSetSpeedLimitRequest(pilz_msgs::SetSpeedLimit::Request& req,
                           pilz_msgs::SetSpeedLimit::Response& /*res*/)
{
  cartesian_speed_limit_ = req.speed_limit;
  return true;
}

}  // namespace pilz_joint_trajectory_controller

#endif  // PILZ_CONTROL_PILZ_JOINT_TRAJECTORY_CONTROLLER_IMPL_H
