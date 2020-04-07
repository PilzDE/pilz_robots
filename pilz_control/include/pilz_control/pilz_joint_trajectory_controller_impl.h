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
#include <joint_trajectory_controller/tolerances.h>

namespace pilz_joint_trajectory_controller
{

static constexpr double SPEED_LIMIT_ACTIVATED{0.25};
static constexpr double SPEED_LIMIT_NOT_ACTIVATED{-1.0};

namespace ph = std::placeholders;

/**
 * @brief Check if a trajectory is in execution at a given uptime of the controller.
 *
 * A trajectory is considered to be in execution at a given time point,
 * if the time point is included in the time interval of at least one segment of the trajectory,
 * or if it lies inside the goal_time_tolerance of at least one segment.
 */
template<class Segment>
bool isTrajectoryExecuted(const std::vector< TrajectoryPerJoint<Segment>>& traj,
                          const ros::Time& curr_uptime)
{
  for (unsigned int joint_index = 0; joint_index < traj.size(); ++joint_index)
  {
    const auto& segment_it = findSegment(traj[joint_index], curr_uptime.toSec());
    const auto& tolerances = segment_it->getTolerances();
    if (segment_it != traj[joint_index].end()
        && curr_uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance)
    {
      return true;
    }
  }
  return false;
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
  robot_model_loader_ = std::make_shared<RobotModelLoader>("robot_description", false);
  auto kinematic_model = robot_model_loader_->getModel();

  using pilz_control::CartesianSpeedMonitor;
  cartesian_speed_monitor_.reset(new CartesianSpeedMonitor(JointTrajectoryController::joint_names_, kinematic_model));
  cartesian_speed_monitor_->init();
  cartesian_speed_limit_ = SPEED_LIMIT_ACTIVATED;

  hold_position_service = controller_nh.advertiseService("hold",
                                                         &PilzJointTrajectoryController::handleHoldRequest,
                                                         this);


  unhold_position_service = controller_nh.advertiseService("unhold",
                                                           &PilzJointTrajectoryController::handleUnHoldRequest,
                                                           this);

  is_executing_service_ = controller_nh.advertiseService("is_executing",
                                                         &PilzJointTrajectoryController::handleIsExecutingRequest,
                                                         this);

  monitor_cartesian_speed_service_ = controller_nh.advertiseService("monitor_cartesian_speed",
                                                        &PilzJointTrajectoryController::handleMonitorCartesianSpeedRequest,
                                                        this);

  stop_traj_builder_ = std::unique_ptr<joint_trajectory_controller::StopTrajectoryBuilder<SegmentImpl> >(new joint_trajectory_controller::StopTrajectoryBuilder<SegmentImpl>(JointTrajectoryController::stop_trajectory_duration_,
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
  auto uptime {JointTrajectoryController::time_data_.readFromRT()->uptime};

  return isTrajectoryExecuted<typename JointTrajectoryController::Segment>(curr_traj, uptime);
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
handleHoldRequest(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& response)
{
  HoldModeListener listener;
  if (mode_->stopEvent(&listener))
  {
    triggerCancellingOfActiveGoal();
    triggerMovementToHoldPosition();
  }

  // Wait till stop motion finished by waiting for hold mode
  listener.wait();

  response.message = "Holding mode enabled";
  response.success = true;
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
handleUnHoldRequest(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& response)
{
  if ( JointTrajectoryController::state_ == JointTrajectoryController::RUNNING
       && mode_->startEvent() )
  {
    response.message = "Unhold mode (default mode) active";
    response.success = true;
    return true;
  }

  response.message = "Could not switch to unhold mode (default mode)";
  response.success = false;
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
  if (mode_->isHolding())
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
    if (!isPlannedCartesianVelocityOK(time_data.period) && mode_->stopEvent())
    {
      stopMotion(time_data.uptime);
    }
    return;
  }
  case TrajProcessingMode::stopping:
  {
    // By construction of the stop trajectory we can exclude that the execution starts in the future
    if ( !isTrajectoryExecuted<typename JointTrajectoryController::Segment>(curr_traj, time_data.uptime) )
    {
      mode_->stopMotionFinishedEvent();
    }
    return;
  }
  case TrajProcessingMode::hold:
    return;
  default:  // LCOV_EXCL_START
    stopMotion(time_data.uptime);
    return;
  }  // LCOV_EXCL_STOP
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
  // It has to be investigated why! -> Probably a threading problem. See also:
  // https://github.com/ros-controls/ros_controllers/issues/174
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::
handleMonitorCartesianSpeedRequest(std_srvs::SetBool::Request& req,
                                   std_srvs::SetBool::Response& res)
{
  cartesian_speed_limit_ = req.data ? SPEED_LIMIT_ACTIVATED : SPEED_LIMIT_NOT_ACTIVATED;
  res.success = true;
  return true;
}

}  // namespace pilz_joint_trajectory_controller

#endif  // PILZ_CONTROL_PILZ_JOINT_TRAJECTORY_CONTROLLER_IMPL_H
