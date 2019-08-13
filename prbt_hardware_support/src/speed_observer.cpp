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
{
  frame_speeds_pub = nh.advertise<FrameSpeeds>(FRAME_SPEEDS_TOPIC_NAME, DEFAULT_QUEUE_SIZE);
  stop_pub = nh.advertise<std_msgs::Empty>(STOP_TOPIC_NAME, DEFAULT_QUEUE_SIZE);
  speeds = std::vector<double>(frames_to_observe.size());
}

void SpeedObserver::startObserving(double frequency)
{
  ros::Rate r(frequency);
  tf::TransformListener listener;
  double cycle_time = 1 / frequency;

  for(auto & frame : frames_to_observe_)
  {
    bool success = false;
    while(!success) {
      try {
        success = listener.canTransform(reference_frame_, frame, ros::Time(0) - ros::Duration(cycle_time));
      }
      catch (std::runtime_error &re) {
        ROS_WARN_STREAM(re.what());
      }
      ROS_WARN("Waiting for transform %s -> %s", reference_frame_.c_str(), frame.c_str());
      ros::Duration(WAITING_TIME_FOR_TRANSFORM_S).sleep();
    }
  }
  ROS_INFO("Observing at %.1fHz", frequency);

  while (ros::ok()){
    try{
      speeds.clear();
      tf::StampedTransform transform_latest, transform_prev;
      for(auto & frame : frames_to_observe_){
        // Look up the twist, averaging over the observation cycle
        ros::Time latest = ros::Time(0);
        listener.lookupTransform(frame, reference_frame_, latest, transform_latest);
        listener.lookupTransform(frame, reference_frame_, latest - ros::Duration(cycle_time), transform_prev);
//        ROS_DEBUG_STREAM("frame, reference_frame_:\nx " << transform.getOrigin().x() <<
//                         "\ny " << transform.getOrigin().y() <<
//                         "\nz " << transform.getOrigin().z());
        double speed = speedFromTwoPoses(
              transform_prev.getOrigin(),
              transform_latest.getOrigin(),
              cycle_time);
        if(!isWithinLimits(speed)){
          ROS_ERROR("Speed %.2f m/s of frame >%s< exceeds limit of %.2f m/s", speed, frame.c_str(), SPEED_LIMIT);
          stop_pub.publish(std_msgs::Empty());
        }
        speeds.push_back(speed);
      }
      frame_speeds_pub.publish(makeFrameSpeedsMessage(speeds));
    }
    catch(const tf2::ExtrapolationException &ex){
      ROS_WARN_STREAM(ex.what());
//      return;
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

//double SpeedObserver::speedFromVelocityVector(const geometry_msgs::Vector3& v)
//{
//  return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
//}

double SpeedObserver::speedFromTwoPoses(tf::Vector3 a, tf::Vector3 b, double t)
{
  return sqrt(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2) + pow(a.z() - b.z(), 2)) / t;
}

bool SpeedObserver::isWithinLimits(const double& speed)
{
  return speed < SPEED_LIMIT;
}

} // namespace prbt_hardware_support

