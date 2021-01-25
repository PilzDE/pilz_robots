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

#include <string>

#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <joint_trajectory_controller/tolerances.h>

namespace pilz_joint_trajectory_controller
{
static constexpr double SPEED_LIMIT_ACTIVATED{ 0.25 };
static constexpr double SPEED_LIMIT_NOT_ACTIVATED{ -1.0 };

static const std::string LIMITS_NAMESPACE{ "limits" };

static const std::string ROBOT_DESCRIPTION_PARAM_NAME{ "/robot_description" };
static const std::string HAS_ACCELERATION_LIMITS_PARAM_NAME{ "/has_acceleration_limits" };
static const std::string MAX_ACCELERATION_PARAM_NAME{ "/max_acceleration" };

static const std::string HOLD_SERVICE_NAME{ "hold" };
static const std::string UNHOLD_SERVICE_NAME{ "unhold" };
static const std::string IS_EXECUTING_SERVICE_NAME{ "is_executing" };
static const std::string MONITOR_CARTESIAN_SPEED_SERVICE_NAME{ "monitor_cartesian_speed" };

static const std::string USER_NOTIFICATION_NOT_IMPLEMENTED_COMMAND_INTERFACE_WARN{
  "The topic interface of the original `joint_trajectory_controller` is deactivated. Please use the action interface "
  "to send goals, that allows monitoring and receiving notifications about cancelled goals. If nonetheless you need "
  "the topic interface feel encouraged to open an issue with this feature request at "
  "https://github.com/PilzDE/pilz_robots/issues so that we can improve your user experience with our product."
};

static const std::string USER_NOTIFICATION_NOT_IMPLEMENTED_COMMAND_INTERFACE_INFO{
  "For the reason behind the deactivation of this interface see "
  "https://github.com/ros-controls/ros_controllers/issues/493). "
  "PR welcome ;-)"
};

namespace ph = std::placeholders;

/**
 * @brief Calculate acceleration in direction of desired movement.
 *
 * @note If the direction of the movement changes (i.e. the velocities have different signs), we have a deceleration
 * from @p old_desired_velocity to 0.0 and an acceleration from 0.0 to @p desired_velocity. In this case the
 * deceleration part is neglected.
 */
inline double calculateAcceleration(const double& desired_velocity, const double& old_desired_velocity,
                                    const ros::Duration& delta_t)
{
  if (desired_velocity > 0.0)
  {
    // neglect deceleration if old_desired_velocity < 0.0
    return (desired_velocity - std::max(0.0, old_desired_velocity)) / delta_t.toSec();
  }
  // neglect deceleration if old_desired_velocity > 0.0
  return (std::min(0.0, old_desired_velocity) - desired_velocity) / delta_t.toSec();
}

/**
 * @brief Check if a trajectory is in execution at a given uptime of the controller.
 *
 * A trajectory is considered to be in execution at a given time point,
 * if the time point is included in the time interval of at least one segment of the trajectory,
 * or if it lies inside the goal_time_tolerance of at least one segment.
 */
template <class Segment>
bool isTrajectoryExecuted(const std::vector<TrajectoryPerJoint<Segment>>& traj, const ros::Time& curr_uptime)
{
  for (unsigned int joint_index = 0; joint_index < traj.size(); ++joint_index)
  {
    const auto& segment_it = findSegment(traj[joint_index], curr_uptime.toSec());
    const auto& tolerances = segment_it->getTolerances();
    if (segment_it != traj[joint_index].end() &&
        curr_uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance)
    {
      return true;
    }
  }
  return false;
};

template <class SegmentImpl, class HardwareInterface>
void PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::makeParamNameWithSuffix(
    std::string& param_name, const std::string& joint_name, const std::string& suffix)
{
  std::stringstream param_name_stream;
  param_name_stream << joint_name << suffix;
  param_name = param_name_stream.str();
}

template <class SegmentImpl, class HardwareInterface>
std::vector<boost::optional<double>>
PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::getJointAccelerationLimits(
    const ros::NodeHandle& nh, const std::vector<std::string>& joint_names)
{
  std::vector<boost::optional<double>> acc_limits(joint_names.size());
  for (unsigned int i = 0; i < joint_names.size(); ++i)
  {
    bool has_acceleration_limits = false;
    std::string has_limits_param_name_to_read;
    makeParamNameWithSuffix(has_limits_param_name_to_read, joint_names.at(i), HAS_ACCELERATION_LIMITS_PARAM_NAME);
    if (!nh.getParam(has_limits_param_name_to_read, has_acceleration_limits))
    {
      throw ros::InvalidParameterException("Failed to get the has_acceleration_limits flag for " + joint_names.at(i) +
                                           " under param name >" + has_limits_param_name_to_read + "<.");
    }

    if (has_acceleration_limits)
    {
      std::string acc_limits_param_name_to_read;
      makeParamNameWithSuffix(acc_limits_param_name_to_read, joint_names.at(i), MAX_ACCELERATION_PARAM_NAME);
      double tmp_limit;
      if (!nh.getParam(acc_limits_param_name_to_read, tmp_limit))
      {
        throw ros::InvalidParameterException("Failed to get the joint acceleration limit for " + joint_names.at(i) +
                                             " under param name >" + acc_limits_param_name_to_read + "<.");
      }
      acc_limits.at(i) = tmp_limit;
    }
  }
  return acc_limits;
}

template <class SegmentImpl, class HardwareInterface>
PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::PilzJointTrajectoryController()
{
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::init(HardwareInterface* hw,
                                                                         ros::NodeHandle& root_nh,
                                                                         ros::NodeHandle& controller_nh)

{
  bool res = JointTrajectoryController::init(hw, root_nh, controller_nh);

  ros::NodeHandle limits_nh(controller_nh, LIMITS_NAMESPACE);
  acceleration_joint_limits_ = getJointAccelerationLimits(limits_nh, JointTrajectoryController::joint_names_);

  using robot_model_loader::RobotModelLoader;
  robot_model_loader_ = std::make_shared<RobotModelLoader>(ROBOT_DESCRIPTION_PARAM_NAME, false);
  auto kinematic_model = robot_model_loader_->getModel();

  using pilz_control::CartesianSpeedMonitor;
  cartesian_speed_monitor_.reset(new CartesianSpeedMonitor(JointTrajectoryController::joint_names_, kinematic_model));
  cartesian_speed_monitor_->init();
  cartesian_speed_limit_ = SPEED_LIMIT_ACTIVATED;

  hold_position_service =
      controller_nh.advertiseService(HOLD_SERVICE_NAME, &PilzJointTrajectoryController::handleHoldRequest, this);

  unhold_position_service =
      controller_nh.advertiseService(UNHOLD_SERVICE_NAME, &PilzJointTrajectoryController::handleUnHoldRequest, this);

  is_executing_service_ = controller_nh.advertiseService(
      IS_EXECUTING_SERVICE_NAME, &PilzJointTrajectoryController::handleIsExecutingRequest, this);

  monitor_cartesian_speed_service_ = controller_nh.advertiseService(
      MONITOR_CARTESIAN_SPEED_SERVICE_NAME, &PilzJointTrajectoryController::handleMonitorCartesianSpeedRequest, this);

  stop_traj_builder_ = std::unique_ptr<joint_trajectory_controller::StopTrajectoryBuilder<SegmentImpl>>(
      new joint_trajectory_controller::StopTrajectoryBuilder<SegmentImpl>(
          JointTrajectoryController::stop_trajectory_duration_, JointTrajectoryController::old_desired_state_));
  stop_traj_velocity_violation_ =
      JointTrajectoryController::createHoldTrajectory(JointTrajectoryController::getNumberOfJoints());

  return res;
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::is_executing()
{
  if (!JointTrajectoryController::isRunning())
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
  auto uptime{ JointTrajectoryController::time_data_.readFromRT()->uptime };

  return isTrajectoryExecuted<typename JointTrajectoryController::Segment>(curr_traj, uptime);
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::handleHoldRequest(
    std_srvs::TriggerRequest&, std_srvs::TriggerResponse& response)
{
  HoldModeListener listener;
  if (mode_->stopEvent(&listener))
  {
    cancelActiveGoal();
    triggerMovementToHoldPosition();
  }

  // Wait till stop motion finished by waiting for hold mode
  listener.wait();

  response.message = "Holding mode enabled";
  response.success = true;
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::handleUnHoldRequest(
    std_srvs::TriggerRequest&, std_srvs::TriggerResponse& response)
{
  if (JointTrajectoryController::isRunning() && mode_->startEvent())
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
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::handleIsExecutingRequest(
    std_srvs::TriggerRequest&, std_srvs::TriggerResponse& response)
{
  response.success = is_executing();
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::updateTrajectoryCommand(
    const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh, std::string* error_string)
{
  if (mode_->isHolding())
  {
    return updateStrategyWhileHolding(msg, gh, error_string);
  }

  // The default case
  return updateStrategyDefault(msg, gh, error_string);
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::updateStrategyDefault(
    const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh, std::string* error_string)
{
  return JointTrajectoryController::updateTrajectoryCommand(msg, gh, error_string);
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::updateStrategyWhileHolding(
    const JointTrajectoryConstPtr&, RealtimeGoalHandlePtr, std::string* error_string)
{
  ROS_WARN_THROTTLE_NAMED(10, this->name_,
                          "Controller received new commands but won't react because it is currently in holding mode.");
  return false;
}

template <class SegmentImpl, class HardwareInterface>
void PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::triggerMovementToHoldPosition()
{
  JointTrajectoryController::setHoldPosition(this->time_data_.readFromRT()->uptime);
}

template <class SegmentImpl, class HardwareInterface>
inline void PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::updateFuncExtensionPoint(
    const typename JointTrajectoryController::Trajectory& curr_traj,
    const typename JointTrajectoryController::TimeData& time_data)
{
  switch (mode_->getCurrentMode())
  {
    case TrajProcessingMode::unhold: {
      if (!isPlannedUpdateOK(time_data.period) && mode_->stopEvent())
      {
        stopMotion(time_data.uptime);
      }
      return;
    }
    case TrajProcessingMode::stopping: {
      // By construction of the stop trajectory we can exclude that the execution starts in the future
      if (!isTrajectoryExecuted<typename JointTrajectoryController::Segment>(curr_traj, time_data.uptime))
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
inline bool
PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::isPlannedUpdateOK(const ros::Duration& period) const
{
  return isPlannedJointAccelerationOK(period) && isPlannedCartesianVelocityOK(period);
}

template <class SegmentImpl, class HardwareInterface>
inline bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::isPlannedJointAccelerationOK(
    const ros::Duration& period) const
{
  for (unsigned int i = 0; i < JointTrajectoryController::getNumberOfJoints(); ++i)
  {
    if (acceleration_joint_limits_.at(i))
    {
      const double& old_velocity = JointTrajectoryController::old_desired_state_.velocity.at(i);
      const double& new_velocity = JointTrajectoryController::desired_state_.velocity.at(i);
      const double& acceleration = calculateAcceleration(new_velocity, old_velocity, period);
      if (acceleration > acceleration_joint_limits_.at(i).value())
      {
        ROS_ERROR_STREAM_NAMED(JointTrajectoryController::name_,
                               "Acceleration limit violated by joint "
                                   << JointTrajectoryController::joint_names_.at(i)
                                   << ". Desired acceleration: " << acceleration
                                   << "rad/s^2, limit: " << acceleration_joint_limits_.at(i) << "rad/s^2.");
        return false;
      }
    }
  }
  return true;
}

template <class SegmentImpl, class HardwareInterface>
inline bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::isPlannedCartesianVelocityOK(
    const ros::Duration& period) const
{
  return (cartesian_speed_monitor_->cartesianSpeedIsBelowLimit(JointTrajectoryController::old_desired_state_.position,
                                                               JointTrajectoryController::desired_state_.position,
                                                               period.toSec(), cartesian_speed_limit_));
}

template <class SegmentImpl, class HardwareInterface>
void PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::stopMotion(const ros::Time& curr_uptime)
{
  abortActiveGoal();

  stop_traj_builder_->setStartTime(JointTrajectoryController::old_time_data_.uptime.toSec())
      ->buildTrajectory(stop_traj_velocity_violation_.get());
  stop_traj_builder_->reset();
  JointTrajectoryController::updateStates(curr_uptime, stop_traj_velocity_violation_.get());

  JointTrajectoryController::curr_trajectory_box_.set(stop_traj_velocity_violation_);
}

template <class SegmentImpl, class HardwareInterface>
inline void PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::cancelActiveGoal()
{
  RealtimeGoalHandlePtr active_goal(JointTrajectoryController::rt_active_goal_);
  if (!active_goal)
  {
    return;
  }
  JointTrajectoryController::rt_active_goal_.reset();
  active_goal->gh_.setCanceled();
  // TODO: Instead of the line above, I actually want to do this:
  //      active_goal->preallocated_result_->error_code =
  //      control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
  //      active_goal->setCanceled(active_goal->preallocated_result_);
  // Unfortunately this does not work because sometimes the goal does not seems to get cancelled.
  // It has to be investigated why! -> Probably a threading problem. See also:
  // https://github.com/ros-controls/ros_controllers/issues/174
}

template <class SegmentImpl, class HardwareInterface>
inline void PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::abortActiveGoal()
{
  RealtimeGoalHandlePtr active_goal(JointTrajectoryController::rt_active_goal_);
  if (!active_goal)
  {
    return;  // LCOV_EXCL_LINE
             // Since the command topic is deactivated it is impossible to violate a limit without an active goal
  }
  JointTrajectoryController::rt_active_goal_.reset();
  active_goal->gh_.setAborted();
  // TODO: Instead of the line above, I actually want to do this:
  //      active_goal->preallocated_result_->error_code =
  //      control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
  //      active_goal->setAborted(active_goal->preallocated_result_);
  // Unfortunately this does not work because sometimes the goal does not seems to get aborted.
  // It has to be investigated why! -> Probably a threading problem. See also:
  // https://github.com/ros-controls/ros_controllers/issues/174
}

template <class SegmentImpl, class HardwareInterface>
bool PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::handleMonitorCartesianSpeedRequest(
    std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  cartesian_speed_limit_ = req.data ? SPEED_LIMIT_ACTIVATED : SPEED_LIMIT_NOT_ACTIVATED;
  res.success = true;
  return true;
}

template <class SegmentImpl, class HardwareInterface>
void PilzJointTrajectoryController<SegmentImpl, HardwareInterface>::trajectoryCommandCB(
    const JointTrajectoryConstPtr& /*msg*/)
{
  ROS_WARN_STREAM_NAMED(this->name_, USER_NOTIFICATION_NOT_IMPLEMENTED_COMMAND_INTERFACE_WARN);
  ROS_INFO_STREAM_NAMED(this->name_, USER_NOTIFICATION_NOT_IMPLEMENTED_COMMAND_INTERFACE_INFO);
}

}  // namespace pilz_joint_trajectory_controller

#endif  // PILZ_CONTROL_PILZ_JOINT_TRAJECTORY_CONTROLLER_IMPL_H
