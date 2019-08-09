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

#include <tf/transform_listener.h>

#include <prbt_hardware_support/speed_observer.h>

namespace prbt_hardware_support
{

static const std::string FRAME_SPEEDS_TOPIC_NAME{"frame_speeds"};
static const uint32_t FRAME_SPEEDS_QUEUE_SIZE{10};

SpeedObserver::SpeedObserver(ros::NodeHandle& nh,
                             std::string& reference_frame,
                             std::vector<std::string>& frames_to_observe)
  : nh_(nh)
  , reference_frame_(reference_frame)
  , frames_to_observe_(frames_to_observe)
{
  frame_speeds_pub = nh.advertise<FrameSpeeds>(FRAME_SPEEDS_TOPIC_NAME, FRAME_SPEEDS_QUEUE_SIZE);
}

void SpeedObserver::startObserving(double frequency)
{
  ros::Rate r(frequency);
  tf::TransformListener listener;

  for(auto & frame : frames_to_observe_){
    ros::spinOnce();
    while(!listener.canTransform(reference_frame_, frame, ros::Time(0)))
    {
      ROS_WARN("Waiting for transform %s -> %s", reference_frame_.c_str(), frame.c_str());
      ros::Duration(1).sleep();
    }
  }

  while (ros::ok())
  {
    try
    {
      std::vector<geometry_msgs::Vector3> velocities = std::vector<geometry_msgs::Vector3>();
      for(auto & frame : frames_to_observe_){
        geometry_msgs::Twist twist;
        // Look up the twist, averaging over the observation cycle
        listener.lookupTwist(reference_frame_, frame, ros::Time(0), ros::Duration(1 / frequency), twist);
        velocities.push_back(twist.linear);
      }
      std::vector<double> speeds;
      speeds.resize(velocities.size());
      std::transform(velocities.begin(), velocities.end(), speeds.begin(), SpeedObserver::speedFromVelocityVector);
      frame_speeds_pub.publish(makeFrameSpeedsMessage(speeds));
    }
    catch(const tf2::ExtrapolationException &ex)
    {
      ROS_ERROR_STREAM(ex.what());
    }

    r.sleep();
  }
}

FrameSpeeds SpeedObserver::makeFrameSpeedsMessage(std::vector<double> speeds)
{
  FrameSpeeds msg;
  msg.header.frame_id = reference_frame_;
  msg.header.seq = seq++;
  msg.header.stamp = ros::Time::now();
  for(auto & n : frames_to_observe_){
    msg.name.push_back(n);
  }
  for(auto & s : speeds){
    msg.speed.push_back(s);
  }
  return msg;
}

double SpeedObserver::speedFromVelocityVector(const geometry_msgs::Vector3& v)
{
  return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

} // namespace prbt_hardware_support

