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

#include <algorithm>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <std_msgs/Float64.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <urdf/model.h>

#include <pilz_utils/get_param.h>

#include <pilz_control/joint_states_speed_observer.h>

namespace pilz_control
{
static const std::string ROBOT_DESCRIPTION_PARAMETER{ "robot_description" };
static const std::string JOINT_STATES_TOPIC_NAME{ "joint_states" };
static const std::string MAX_FRAME_SPEED_TOPIC_NAME{ "max_frame_speed" };
static const std::string MONITORED_LINK_NAMES_PARAMETER{ "monitored_link_names" };

JointStatesSpeedObserver::JointStatesSpeedObserver(const ros::NodeHandle& nh) : nh_(nh)
{
  setupKinematics();

  frames_to_observe_ = pilz_utils::getParam<std::vector<std::string>>(nh_, MONITORED_LINK_NAMES_PARAMETER);
  assert(validateLinkNames());

  max_frame_speed_pub_ = nh_.advertise<std_msgs::Float64>(MAX_FRAME_SPEED_TOPIC_NAME, 10);
  joint_states_sub_ = nh_.subscribe(JOINT_STATES_TOPIC_NAME, 10, &JointStatesSpeedObserver::jointStatesCallback, this);

  ROS_INFO("Speed observation is running now");
}

void JointStatesSpeedObserver::setupKinematics()
{
  ROS_INFO("Loading model");
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAMETER, false));
  kinematic_model_ = robot_model_loader_->getModel();
  ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());

  kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
  kinematic_state_->setToDefaultValues();
  ROS_INFO("Done loading model");
}

bool JointStatesSpeedObserver::validateLinkNames()
{
  return std::all_of(frames_to_observe_.begin(), frames_to_observe_.end(),
                     [this](const std::string& name) { return kinematic_model_->hasLinkModel(name); });
}

void JointStatesSpeedObserver::jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  bool computed_new_speed{ false };

  ros::Time time_stamp{ msg->header.stamp };

  kinematic_state_->setVariablePositions(msg->name, msg->position);

  double max_frame_speed = 0.0;

  for (const auto& frame : frames_to_observe_)
  {
    auto tf = kinematic_state_->getGlobalLinkTransform(frame);

    if (previous_tfs_.find(frame) != previous_tfs_.end())
    {
      auto distance_cartesian = (tf.translation() - previous_tfs_.at(frame).translation()).norm();
      auto time_distance = (time_stamp - previous_time_stamp_).toSec();

      max_frame_speed = std::max(max_frame_speed, distance_cartesian / time_distance);
      computed_new_speed = true;
    }

    previous_tfs_[frame] = tf;
  }

  previous_time_stamp_ = time_stamp;

  if (computed_new_speed)
  {
    publishMaxFrameSpeed(max_frame_speed);
  }
}

void JointStatesSpeedObserver::publishMaxFrameSpeed(const double& speed)
{
  std_msgs::Float64 msg;
  msg.data = speed;
  max_frame_speed_pub_.publish(msg);
}

}  // namespace pilz_control

/**
 * @brief Create speed observer and spin.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_speed_observer");
  ros::NodeHandle nh;

  pilz_control::JointStatesSpeedObserver observer(nh);

  ros::spin();

  return EXIT_SUCCESS;
}
