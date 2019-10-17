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
#include <functional>
#include <std_srvs/Trigger.h>
#include <stdexcept>
#include <tf2/convert.h>

#include <prbt_hardware_support/speed_observer.h>
#include <prbt_hardware_support/wait_for_service.h>

namespace prbt_hardware_support
{
static const std::string FRAME_SPEEDS_TOPIC_NAME{ "frame_speeds" };
static const std::string HOLD_SERVICE{ "manipulator_joint_trajectory_controller/hold" };

SpeedObserver::SpeedObserver(ros::NodeHandle& nh, std::string& reference_frame,
                             std::vector<std::string>& frames_to_observe, bool simulation)
  : nh_(nh), reference_frame_(reference_frame), frames_to_observe_(frames_to_observe)
  , simulation_(simulation)
{
  frame_speeds_pub_ = nh.advertise<FrameSpeeds>(FRAME_SPEEDS_TOPIC_NAME, DEFAULT_QUEUE_SIZE);
  if (!simulation)
  {
    waitForService(HOLD_SERVICE);
    hold_client_ = nh.serviceClient<std_srvs::Trigger>(HOLD_SERVICE);
  }

  tf_sub_ = nh_.subscribe("/tf", 10, &SpeedObserver::tfCallback, this);
}

std::pair<tf2::Vector3, ros::Time> SpeedObserver::getLatestPose(const std::string& frame) const
{
  tf2::Vector3 v;
  ros::Time t;

  geometry_msgs::TransformStamped transform{ tf_buffer_.lookupTransform(reference_frame_, frame, ros::Time(0)) };
  tf2::fromMsg(transform.transform.translation, v);
  t = transform.header.stamp;
  return std::pair<tf2::Vector3, ros::Time>(v, t);
}

void SpeedObserver::triggerStop1()
{
  if (!simulation_)
  {
    std_srvs::Trigger hold_srv;
    bool call_success = hold_client_.call(hold_srv);
    if (!call_success)
    {
      ROS_ERROR_STREAM("No success calling service: " << hold_client_.getService());
    }
    else if (!hold_srv.response.success)
    {
      ROS_ERROR_STREAM("Service: " << hold_client_.getService() << " failed with error message:\n"
                                   << hold_srv.response.message);
    }
  }
}

bool SpeedObserver::setSpeedLimitCb(SetSpeedLimit::Request& req, SetSpeedLimit::Response& res)
{
  ROS_DEBUG_STREAM("setSpeedLimitCb " << req.speed_limit);
  current_speed_limit_ = req.speed_limit;
  return true;
}

void SpeedObserver::tfCallback(const tf2_msgs::TFMessageConstPtr &msg)
{
  for (const auto& transform : msg->transforms)
  {
    tf_buffer_.setTransform(transform, "speed_observer");
  }

  bool computed_new_frame_speeds{false};

  for (const auto& frame : frames_to_observe_)
  {
    const auto curr_pose_data = getLatestPose(frame);
    const auto &curr_pose = curr_pose_data.first;
    const auto &curr_time_stamp = curr_pose_data.second;

    if (initial_callback_)
    {
      previous_poses_[frame] = tf2::Vector3(curr_pose);
      previous_time_stamps_[frame] = curr_time_stamp;
      continue;
    }

    double curr_speed{0.0};
    if ((curr_time_stamp - previous_time_stamps_.at(frame)).toSec() > TIME_INTERVAL_EPSILON_S)
    {
      curr_speed = speedFromTwoPoses(previous_poses_.at(frame),
                                     curr_pose,
                                     (curr_time_stamp - previous_time_stamps_.at(frame)).toSec());
      computed_new_frame_speeds = true;
    }
    else
    {
      curr_speed = previous_speeds_.at(frame);
      ROS_DEBUG("Skip speed computation for frame >%s<.", frame.c_str());
    }
    if (!isWithinLimit(curr_speed))
    {
      ROS_ERROR("Speed %.2f m/s of frame >%s< exceeds limit of %.2f m/s", curr_speed, frame.c_str(),
                current_speed_limit_);
      triggerStop1();
    }

    previous_speeds_[frame] = curr_speed;
    previous_poses_[frame] = tf2::Vector3(curr_pose);
    previous_time_stamps_[frame] = curr_time_stamp;
  }

  if (initial_callback_)
  {
    initial_callback_ = false;
  }
  else if (computed_new_frame_speeds)
  {
    publishFrameSpeedsMessage();
  }
}

void SpeedObserver::publishFrameSpeedsMessage()
{
  static uint32_t seq{ 0 };
  FrameSpeeds msg;
  msg.header.frame_id = reference_frame_;
  msg.header.seq = seq++;

  typedef std::pair<std::string, ros::Time> TimeStampsElem;
  const auto min_el_it = std::min_element(previous_time_stamps_.begin(),
                                          previous_time_stamps_.end(),
                                          [](const TimeStampsElem &el1, const TimeStampsElem &el2)
                                          { return el1.second < el2.second; });
  msg.header.stamp = min_el_it->second;

  for (const auto& s : previous_speeds_)
  {
    msg.name.push_back(s.first);
    msg.speed.push_back(s.second);
  }

  frame_speeds_pub_.publish(msg);
}

}  // namespace prbt_hardware_support
