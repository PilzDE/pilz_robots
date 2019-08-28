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

#include <tf2/convert.h>
#include <std_srvs/SetBool.h>
#include <functional>
#include <algorithm>
#include <stdexcept>

#include <prbt_hardware_support/speed_observer.h>

namespace prbt_hardware_support
{

static const std::string FRAME_SPEEDS_TOPIC_NAME{"frame_speeds"};
static const std::string STO_SERVICE{"safe_torque_off"};

SpeedObserver::SpeedObserver(ros::NodeHandle& nh,
                             std::string& reference_frame,
                             std::vector<std::string>& frames_to_observe)
  : nh_(nh)
  , reference_frame_(reference_frame)
  , frames_to_observe_(frames_to_observe)
{
  frame_speeds_pub_ = nh.advertise<FrameSpeeds>(FRAME_SPEEDS_TOPIC_NAME, DEFAULT_QUEUE_SIZE);
  sto_client_ = nh.serviceClient<std_srvs::SetBool>(STO_SERVICE, DEFAULT_QUEUE_SIZE);
}

void SpeedObserver::waitTillTFReady(const std::string& frame,
                                    const ros::Time& now,
                                    const unsigned short int max_num_retries) const
{
  unsigned short int retries {0};

  while( !terminate_ && (retries < max_num_retries) &&  !tf_buffer_.canTransform(reference_frame_, frame, now) )
  {
    ros::spinOnce();
    ROS_WARN("Waiting for transform %s -> %s", reference_frame_.c_str(), frame.c_str());
    ros::Duration(WAITING_TIME_FOR_TRANSFORM_S).sleep();
    ++retries;
  }

  if (retries >= max_num_retries)
  {
    ROS_WARN("Waited for transform %s -> %s for too long.",
             reference_frame_.c_str(), frame.c_str());
    throw std::runtime_error("Exceeded maximum number of retries.");
  }

  if (terminate_)
  {
    throw std::runtime_error("Terminate flag is true");
  }
}

tf2::Vector3 SpeedObserver::getPose(const std::string& frame, const ros::Time& now) const
{
  tf2::Vector3 v;
  geometry_msgs::TransformStamped transform {tf_buffer_.lookupTransform(reference_frame_, frame, now)};
  tf2::fromMsg(transform.transform.translation, v);
  return tf2::Vector3(v);
}

void SpeedObserver::startObserving(double frequency)
{ 
  ros::Time now = ros::Time(0);
  std::map<std::string, tf2::Vector3> previous_poses;
  try
  {
    for(const auto& frame : frames_to_observe_)
    {
      waitTillTFReady(frame, now);
      previous_poses[frame] = getPose(frame, now);
    }
  }
  catch (std::runtime_error &re)
  {
    ROS_WARN_STREAM(re.what());
    return;
  }

  ROS_INFO("Observing at %.1fHz", frequency);
  ros::Rate r(frequency);

  tf2::Vector3 curr_pos;
  std::vector<double> speeds;
  speeds.reserve(frames_to_observe_.size());
  ros::Time previous_t = now;

  // Starting the observer loop
  while (ros::ok() && !terminate_)
  {
    speeds.clear();
    now = ros::Time::now();
    for(const auto& frame : frames_to_observe_)
    {
      if (terminate_)
      {
        return;
      }
      try
      {
        waitTillTFReady(frame, now);
        curr_pos = getPose(frame, now);
      }
      catch(std::runtime_error &ex)
      {
        ROS_ERROR_STREAM(ex.what());
        return;
      }

      double curr_speed {speedFromTwoPoses(previous_poses.at(frame), curr_pos, (now - previous_t).toSec())};
      if(!isWithinLimits(curr_speed))
      {
        ROS_ERROR("Speed %.2f m/s of frame >%s< exceeds limit of %.2f m/s", curr_speed, frame.c_str(), SPEED_LIMIT);
        triggerStop1();
      }

      speeds.emplace_back(curr_speed);
      previous_poses[frame] = tf2::Vector3(curr_pos);
    }
    previous_t = now;
    frame_speeds_pub_.publish(createFrameSpeedsMessage(speeds));
    if(terminate_)
    {
      return;
    }
    r.sleep();
  }
}

FrameSpeeds SpeedObserver::createFrameSpeedsMessage(const std::vector<double>& speeds) const
{
  static uint32_t seq{0};
  FrameSpeeds msg;
  msg.header.frame_id = reference_frame_;
  msg.header.seq = seq++;
  msg.header.stamp = ros::Time::now();
  for(const auto & n : frames_to_observe_)
  {
    msg.name.push_back(n);
  }
  for(const auto & s : speeds)
  {
    msg.speed.push_back(s);
  }
  return msg;
}

void SpeedObserver::triggerStop1()
{
  std_srvs::SetBool sto_srv;
  sto_srv.request.data = true;
  bool call_success = sto_client_.call(sto_srv);
  if (!call_success)
  {
    ROS_ERROR_STREAM("No success calling service: " << sto_client_.getService());
  }

  if (!sto_srv.response.success)
  {
    ROS_ERROR_STREAM("Service: " << sto_client_.getService()
                     << " failed with error message:\n"
                     << sto_srv.response.message);
  }
}

} // namespace prbt_hardware_support

