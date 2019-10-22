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

#include <algorithm>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <urdf/model.h>

#include <prbt_hardware_support/FrameSpeeds.h>
#include <prbt_hardware_support/joint_states_speed_observer.h>

namespace prbt_hardware_support
{

bool hasOnlyFixedParentJoints(const urdf::LinkSharedPtr &link)
{
  auto parent_link {link};
  while ( (parent_link->parent_joint != nullptr) && (parent_link->parent_joint->type == urdf::Joint::FIXED) )
  {
    parent_link = parent_link->getParent();
  }
  return parent_link->parent_joint == nullptr;
}

JointStatesSpeedObserver::JointStatesSpeedObserver(ros::NodeHandle nh)
  : nh_(nh)
{
  setup();

  frame_speeds_pub_ = nh_.advertise<FrameSpeeds>("joint_states_frame_speeds", 10);
  joint_states_sub_ = nh_.subscribe("joint_states", 10, &JointStatesSpeedObserver::jointStatesCallback, this);
}

void JointStatesSpeedObserver::setup()
{
  urdf::Model model;
  model.initParam("robot_description");
  ROS_INFO_STREAM("reference_frame: " << model.getRoot()->name);
  reference_frame_ = model.getRoot()->name;
  std::vector<urdf::LinkSharedPtr> links;
  model.getLinks(links);
  ROS_INFO_STREAM("Received the following frames to observer from urdf:");
  for (const auto& link : links)
  {
    // exclude frames which cannot move
    if (!hasOnlyFixedParentJoints(link))
    {
      ROS_INFO_STREAM(" - " << link->name);
      frames_to_observe_.push_back(link->name);
    }
  }

  ROS_INFO("Loading model");
  bool load_kinematics_solvers = true; // OTHERWISE warning TODO investigate what is best todo here. Check if loaded?
  robot_model_loader_.reset( new robot_model_loader::RobotModelLoader("robot_description", load_kinematics_solvers) );
  kinematic_model_ = robot_model_loader_->getModel();
  ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());

  kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
  kinematic_state_->setToDefaultValues();
  ROS_INFO("Done loading model");
}

void JointStatesSpeedObserver::jointStatesCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  bool computed_new_frame_speeds{ false };

  ros::Time time_stamp{ msg->header.stamp };

  kinematic_state_->setVariablePositions(msg->position);  // TODO check that correct order

  for(const auto &frame : frames_to_observe_)
  {
    auto tf = kinematic_state_->getGlobalLinkTransform(frame);

    if (previous_tfs_.find(frame) != previous_tfs_.end())
    {
      auto distance_cartesian = (tf.translation() - previous_tfs_.at(frame).translation()).norm();
      auto time_distance = (time_stamp - previous_time_stamp_).toSec();

      if (time_distance > 1e-16)
      {
        computed_new_frame_speeds = true;
        previous_speeds_[frame] = distance_cartesian / time_distance;
      }
      else
      {
        ROS_INFO("Skipping speed computation for frame >%s<.", frame.c_str());
      }
    }

    previous_tfs_[frame] = tf;
  }

  previous_time_stamp_ = time_stamp;

  if (computed_new_frame_speeds)
  {
    publishFrameSpeedsMessage();
  }
}

void JointStatesSpeedObserver::publishFrameSpeedsMessage()
{
  static uint32_t seq{ 0 };
  FrameSpeeds msg;
  msg.header.frame_id = reference_frame_;
  msg.header.seq = seq++;
  msg.header.stamp = previous_time_stamp_;

  for (const auto& s : previous_speeds_)
  {
    msg.name.push_back(s.first);
    msg.speed.push_back(s.second);
  }

  frame_speeds_pub_.publish(msg);
}

}  // namespace prbt_hardware_support

/**
 * @brief Create speed observer and spin.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_speed_observer");
  ros::NodeHandle nh;

  prbt_hardware_support::JointStatesSpeedObserver observer(nh);

  ros::spin();

  return EXIT_SUCCESS;
}
