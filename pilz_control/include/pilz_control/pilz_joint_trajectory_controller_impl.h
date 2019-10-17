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

  ROS_ERROR("Loading model");
  bool load_kinematics_solvers = true; // OTHERWISE warning TODO investigate what is best todo here. Check if loaded?
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description", load_kinematics_solvers));
  kinematic_model_ = robot_model_loader_->getModel();
  ROS_ERROR("Model frame: %s", kinematic_model_->getModelFrame().c_str());

  kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
  kinematic_state_->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup("manipulator");
  ROS_ERROR("Done loading model");

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
  // TODO Check that first point matches current position and that time_from start equals zero for the first point
  std::vector<trajectory_msgs::JointTrajectoryPoint> points = msg->points;

  // Handle time_from_start greater than zero
  const double epsilon{1e-12};
  if (points.size() > 0 && points.at(0).time_from_start.toSec() > epsilon)
  {
    // prepare new trajectory point
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(0.0);
    point.positions = std::vector<double>(points.at(0).positions.size(), 0.0);

    // insert current position at beginning
    for (const auto &jh : JointTrajectoryController::joints_)
    {
      point.positions.push_back(jh.getPosition()); // not sure about order
    }
    points.insert(points.begin(), point);
  }

  std::vector<std::string> frames_to_observe;
  auto links = kinematic_model_->getLinkModels();
  ROS_DEBUG_STREAM("Received the following frames to observer from urdf:");
  for (const auto& link : links)
  {
    if(!link->parentJointIsFixed()) // Not sure about this...
    {
      ROS_ERROR_STREAM(" - " << link->getName());
      frames_to_observe.push_back(link->getName());
    }
  }

  size_t counter{0};
  ROS_ERROR_STREAM("Checking trajectory with " << points.size() << " points");
  auto start_t = ros::Time::now();

  // Check trajectory
  auto first_violation_point = std::adjacent_find(points.begin(),
                                                  points.end(),
                                                  [this, frames_to_observe, &counter](const trajectory_msgs::JointTrajectoryPoint& p1, const trajectory_msgs::JointTrajectoryPoint& p2) -> bool
                                                  {

                                                    for(const auto &frame : frames_to_observe)
                                                    {                                                                                                          // Calculate the distance
                                                      kinematic_state_->setVariablePositions(p1.positions);  // TODO check that correct order
                                                      auto p1_cart = kinematic_state_->getGlobalLinkTransform(frame); // TODO needs to be done for all frames

                                                      // Calculate the distance
                                                      kinematic_state_->setVariablePositions(p2.positions);
                                                      auto p2_cart = kinematic_state_->getGlobalLinkTransform(frame); // TODO needs to be done for all frames

                                                      auto distance_cartesian = (p2_cart.translation() - p1_cart.translation()).squaredNorm();
                                                      auto time_distance = p2.time_from_start - p1.time_from_start;

                                                      auto velocity = distance_cartesian / time_distance.toSec();

                                                      counter++;

                                                      constexpr double max_vel = 0.025;
                                                      if(velocity > max_vel)
                                                      {
                                                        ROS_ERROR_STREAM("Velocity between " << p1.time_from_start << "s and " << p2.time_from_start << "s is " << velocity << " (max. allowed " << max_vel << "m/s)");
                                                        return true;
                                                      }
                                                    }

                                                    return false;
                  });

  auto duration_ms = (ros::Time::now() - start_t).toSec() * 1000;

  ROS_ERROR_STREAM("Total checks: " << counter << " took " << duration_ms << "ms");

  typedef joint_trajectory_controller::InitJointTrajectoryOptions<Trajectory> Options;
  Options options;
  options.error_string = error_string;
  std::string error_string_tmp;

  if (first_violation_point != points.end())
  {
    error_string_tmp = "Velocity violated";
    ROS_ERROR_STREAM_NAMED(JointTrajectoryController::name_, error_string_tmp);
    options.setErrorString(error_string_tmp);
    return false;
  }


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
