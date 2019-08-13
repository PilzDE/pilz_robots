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
#include <std_msgs/Empty.h>

#include <prbt_hardware_support/speed_observer.h>

namespace prbt_hardware_support
{

static const std::string FRAME_SPEEDS_TOPIC_NAME{"frame_speeds"};
static const std::string STOP_TOPIC_NAME{"stop"};
static const uint32_t DEFAULT_QUEUE_SIZE{10};
static const uint32_t WAITING_TIME_FOR_TRANSFORM_S{1};
static const double SPEED_LIMIT{.25};

SpeedObserver::SpeedObserver(ros::NodeHandle& nh,
                             std::string& reference_frame,
                             std::vector<std::string>& frames_to_observe)
  : nh_(nh)
  , reference_frame_(reference_frame)
  , frames_to_observe_(frames_to_observe)
  , tf_buffer()
{
  frame_speeds_pub = nh.advertise<FrameSpeeds>(FRAME_SPEEDS_TOPIC_NAME, DEFAULT_QUEUE_SIZE);
  stop_pub = nh.advertise<std_msgs::Empty>(STOP_TOPIC_NAME, DEFAULT_QUEUE_SIZE);
  speeds = std::vector<double>(0);
}

void SpeedObserver::startObserving(double frequency)
{
  ros::Rate r(frequency);
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::TransformStamped transform;
  tf2::Vector3 v;
  ros::Time now = ros::Time(0);

  // Making sure tf is ready
  for(auto & frame : frames_to_observe_)
  {
    bool success = tf_buffer.canTransform(reference_frame_, frame, now);
    while(!success) {
      ROS_WARN("Waiting for transform %s -> %s", reference_frame_.c_str(), frame.c_str());
      ros::Duration(WAITING_TIME_FOR_TRANSFORM_S).sleep();
      try {
        success = tf_buffer.canTransform(reference_frame_, frame, now);
      }
      catch(tf2::TransformException &ex){
        ROS_ERROR_STREAM(ex.what());
        return;
      }
      catch (std::runtime_error &re) {
        ROS_WARN_STREAM(re.what());
      }
    }
    transform = tf_buffer.lookupTransform(reference_frame_, frame, now);
    tf2::fromMsg(transform.transform.translation, v);
    previous_poses[frame] = tf2::Vector3(v);
  }
  previous_t = now;
  ROS_INFO("Observing at %.1fHz", frequency);

  // Starting the observer loop
  while (ros::ok()){
    now = ros::Time::now();
    try{
      speeds.clear();
      for(auto & frame : frames_to_observe_){
        while(!tf_buffer.canTransform(reference_frame_, frame, now))
        {
          ros::spinOnce();
        }
        transform = tf_buffer.lookupTransform(reference_frame_, frame, now);
        tf2::fromMsg(transform.transform.translation, v);
        double speed = speedFromTwoPoses(
              previous_poses[frame],
              v,
              (now - previous_t).toSec());
        previous_poses[frame] = tf2::Vector3(v);
        if(!isWithinLimits(speed)){
          ROS_ERROR("Speed %.2f m/s of frame >%s< exceeds limit of %.2f m/s", speed, frame.c_str(), SPEED_LIMIT);
          stop_pub.publish(std_msgs::Empty());
        }
        speeds.push_back(speed);
      }
      previous_t = now;
      frame_speeds_pub.publish(makeFrameSpeedsMessage(speeds));
    }
    catch(tf2::TransformException &ex){
      ROS_ERROR_STREAM(ex.what());
      return;
    }
    r.sleep();
  }
}

FrameSpeeds SpeedObserver::makeFrameSpeedsMessage(std::vector<double>& speeds){
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

double SpeedObserver::speedFromTwoPoses(tf2::Vector3 a, tf2::Vector3 b, double t)
{
  double d = tf2::tf2Distance(a, b);
  return d / t;
}

bool SpeedObserver::isWithinLimits(const double& speed)
{
  return speed < SPEED_LIMIT;
}

} // namespace prbt_hardware_support

