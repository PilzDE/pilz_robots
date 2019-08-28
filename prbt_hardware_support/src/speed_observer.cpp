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

#include <prbt_hardware_support/speed_observer.h>

namespace prbt_hardware_support
{

static const std::string FRAME_SPEEDS_TOPIC_NAME{"frame_speeds"};
static const std::string STO_SERVICE{"safe_torque_off"};
static const uint32_t DEFAULT_QUEUE_SIZE{10};
static const uint32_t WAITING_TIME_FOR_TRANSFORM_S{1};
static const uint32_t WAITING_FOR_TRANSFORM_RETRIES{20};
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
  sto_client_ = nh.serviceClient<std_srvs::SetBool>(STO_SERVICE, DEFAULT_QUEUE_SIZE);
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
  while (ros::ok() & !terminate){
    now = ros::Time::now();
    try{
      speeds.clear();
      for(auto & frame : frames_to_observe_){
        uint32_t retries = 0;
        while(!tf_buffer.canTransform(reference_frame_, frame, now))
        {
          ros::spinOnce();
          ros::Duration(WAITING_TIME_FOR_TRANSFORM_S).sleep();
          if(retries > WAITING_FOR_TRANSFORM_RETRIES){
            ROS_WARN("Waited for transform %s -> %s for too long.", reference_frame_.c_str(), frame.c_str());
            return;
          }
          retries++;
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
          triggerStop1();
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

// This method is only needed for the unittest
void SpeedObserver::terminateNow(){
  ROS_DEBUG("terminateNow");
  terminate = true;
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

