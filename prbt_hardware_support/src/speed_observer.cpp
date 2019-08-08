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
  ros::Rate r(frequency); // 10 hz
  tf::TransformListener listener;

  while (ros::ok())
  {
    ros::spinOnce();
    if(!listener.canTransform(reference_frame_, frames_to_observe_[0], ros::Time(0)))
    {
      ROS_ERROR("Cannot transform!");
      continue;
    }

    try
    {
      geometry_msgs::Twist twist;
      listener.lookupTwist(reference_frame_, frames_to_observe_[0], ros::Time(0), ros::Duration(0.1), twist);
      std::vector<geometry_msgs::Vector3> speeds = std::vector<geometry_msgs::Vector3>({twist.linear});
      frame_speeds_pub.publish(makeFrameSpeedsMessage(speeds));
    }
    catch(const tf2::ExtrapolationException &ex)
    {
      ROS_ERROR_STREAM(ex.what());
    }



    r.sleep();
  }
}

FrameSpeeds SpeedObserver::makeFrameSpeedsMessage(std::vector<geometry_msgs::Vector3> velocities)
{
  FrameSpeeds msg;
  msg.header.frame_id = reference_frame_;
  msg.header.seq = seq++;
  msg.header.stamp = ros::Time::now();

  for(auto & n : frames_to_observe_){
    msg.name.push_back(n);
  }

  for(auto & v : velocities){
    msg.speed.push_back(v.x);  // TODO: https://eigen.tuxfamily.org/dox/classEigen_1_1MatrixBase.html#a196c4ec3c8ffdf5bda45d0f617154975
  }

  return msg;
}

} // namespace prbt_hardware_support

