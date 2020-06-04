/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PILZ_CONTROL_JOINT_STATES_SPEED_OBSERVER_H
#define PILZ_CONTROL_JOINT_STATES_SPEED_OBSERVER_H

#include <utility>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <sensor_msgs/JointState.h>

namespace pilz_control
{
/**
 * @brief Computes frame speeds from joint_states data.
 */
class JointStatesSpeedObserver
{
public:
  JointStatesSpeedObserver(const ros::NodeHandle& nh);

private:
  void setupKinematics();
  bool validateLinkNames();
  void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
  void publishMaxFrameSpeed(const double& speed);

private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_states_sub_;
  ros::Publisher max_frame_speed_pub_;

  std::string reference_frame_;
  std::vector<std::string> frames_to_observe_;
  ros::Time previous_time_stamp_;
  std::map<std::string, Eigen::Isometry3d> previous_tfs_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr kinematic_model_;
  robot_state::RobotStatePtr kinematic_state_;  // Needed for foward calculations
};

}  // namespace pilz_control

#endif  // PILZ_CONTROL_JOINT_STATES_SPEED_OBSERVER_H
