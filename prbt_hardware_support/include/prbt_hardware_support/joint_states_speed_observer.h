/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
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

#ifndef PRBT_HARDWARE_SUPPORT_JOINT_STATES_SPEED_OBSERVER_H
#define PRBT_HARDWARE_SUPPORT_JOINT_STATES_SPEED_OBSERVER_H

#include <utility>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <sensor_msgs/JointState.h>

#include <prbt_hardware_support/FrameSpeeds.h>

namespace prbt_hardware_support
{

/**
 * @brief Computes frame speeds from joint_states data.
 */
class JointStatesSpeedObserver
{
public:
  JointStatesSpeedObserver(ros::NodeHandle nh);

private:
  void setup();
  void jointStatesCallback(const sensor_msgs::JointStateConstPtr &msg);
  void publishFrameSpeedsMessage();

private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_states_sub_;
  ros::Publisher frame_speeds_pub_;

  //! Reference frame for all speeds
  std::string reference_frame_;
  //! All frames to observe
  std::vector<std::string> frames_to_observe_;
  //! time stamp of previous callback
  ros::Time previous_time_stamp_;
  //! poses of previous callback
  std::map<std::string, Eigen::Isometry3d> previous_tfs_;
  //! speeds computed in previous callback
  std::map<std::string, double> previous_speeds_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr kinematic_model_;
  robot_state::RobotStatePtr kinematic_state_; // Needed for foward calculations
};

}

#endif  // PRBT_HARDWARE_SUPPORT_JOINT_STATES_SPEED_OBSERVER_H
