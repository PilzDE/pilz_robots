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
static const std::string HOLD_SERVICE{ "manipulator_joint_trajectory_conroller/hold" };

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

void SpeedObserver::waitUntillCanTransform(const std::string& frame, const ros::Time& time,
                                           const unsigned short int max_num_retries) const
{
  unsigned short int retries{ 0 };

  while (!terminate_ && (retries < max_num_retries) && !tf_buffer_.canTransform(reference_frame_, frame, time))
  {
    ros::spinOnce();
    if (retries > 0)  // when trying for the first time, we do not warn the user
      ROS_WARN("Waiting for transform %s -> %s", reference_frame_.c_str(), frame.c_str());
    ros::Duration(WAITING_TIME_FOR_TRANSFORM_S).sleep();
    ++retries;
  }

  if (retries >= max_num_retries)
  {
    ROS_ERROR("Waited for transform %s -> %s too long.", reference_frame_.c_str(), frame.c_str());
    throw std::runtime_error("Exceeded maximum number of retries.");
  }

  if (terminate_)
  {
    throw std::runtime_error("Terminate flag is true"); // LCOV_EXCL_LINE Flag only needed for tests, therefore, excluded from line coverage.
  }
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

void SpeedObserver::startObserving(const double frequency, const unsigned int allowed_missed_calculations)
{
  std::map<std::string, tf2::Vector3> previous_poses;
  std::map<std::string, ros::Time> previous_time_stamps;
  for (const auto& frame : frames_to_observe_)
  {
    waitUntillCanTransform(frame, ros::Time(0));

    auto pose_data = getLatestPose(frame);
    previous_poses[frame] = pose_data.first;
    previous_time_stamps[frame] = pose_data.second;
    missed_calculations_[frame] = 0;
  }

  ros::Rate r(frequency);
  ROS_INFO("Observing at %.1fHz", frequency);

  tf2::Vector3 curr_pos;
  std::map<std::string, double> speeds;

  // Starting the observer loop
  while (ros::ok() && !terminate_)
  {
    speeds.clear();
    for (const auto& frame : frames_to_observe_)
    {
      if (terminate_)
      {
        return;  // LCOV_EXCL_LINE Flag only needed for tests, therefore, excluded from line coverage.
      }
        
      const auto curr_pose_data = getLatestPose(frame);
      const auto &curr_pose = curr_pose_data.first;
      const auto &curr_time_stamp = curr_pose_data.second;

      if (std::abs((ros::Time::now() - curr_time_stamp).toSec()) > 2.0/frequency)
      {
        ROS_WARN_STREAM("Latest transform of frame " << frame << " is too old. ");
        ++missed_calculations_[frame];
        ROS_WARN_STREAM("Missed calculations for frame " << frame << ": " << missed_calculations_[frame]);
        if (missed_calculations_[frame] > allowed_missed_calculations)
        {
          ROS_ERROR_STREAM("Could not compute frame speed for frame " << frame
                           << " for " << allowed_missed_calculations << " times."
                           << " Triggering Stop1.");
          triggerStop1();
          missed_calculations_[frame] = 0;
        }
      }
      else
      {
        double curr_speed{0.0};
        if ((curr_time_stamp - previous_time_stamps.at(frame)).toSec() > TIME_INTERVAL_EPSILON_S)
        {
          curr_speed = speedFromTwoPoses(previous_poses.at(frame),
                                         curr_pose,
                                         (curr_time_stamp - previous_time_stamps.at(frame)).toSec());
        }
        else
        {
          ROS_WARN("Time interval too small for speed computation.");
        }
        if (!isWithinLimit(curr_speed))
        {
          ROS_ERROR("Speed %.2f m/s of frame >%s< exceeds limit of %.2f m/s", curr_speed, frame.c_str(),
                    current_speed_limit_);
          triggerStop1();
        }

        speeds[frame] = curr_speed;
        previous_poses[frame] = tf2::Vector3(curr_pose);
        previous_time_stamps[frame] = curr_time_stamp;
      }
    }
    frame_speeds_pub_.publish(createFrameSpeedsMessage(speeds));
    ros::spinOnce();
    if (!terminate_)
    {
      r.sleep();
    }
  }
}

FrameSpeeds SpeedObserver::createFrameSpeedsMessage(const std::map<std::string, double>& speeds) const
{
  static uint32_t seq{ 0 };
  FrameSpeeds msg;
  msg.header.frame_id = reference_frame_;
  msg.header.seq = seq++;
  msg.header.stamp = ros::Time::now();
  for (const auto& s : speeds)
  {
    msg.name.push_back(s.first);
    msg.speed.push_back(s.second);
  }
  return msg;
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
