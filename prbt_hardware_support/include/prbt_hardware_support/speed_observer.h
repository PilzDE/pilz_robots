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

#ifndef SPEED_OBSERVER_H
#define SPEED_OBSERVER_H

#include <atomic>

#include <ros/node_handle.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <prbt_hardware_support/FrameSpeeds.h>
#include <prbt_hardware_support/SetSpeedLimit.h>

namespace prbt_hardware_support {

class SpeedObserver {
public:
  SpeedObserver(ros::NodeHandle &nh, std::string &reference_frame,
                std::vector<std::string> &frames_to_observe);

public:
  void startObserving(double frequency);
  bool setSpeedLimitCb(SetSpeedLimit::Request& req, SetSpeedLimit::Response& res);
  void terminateNow();

private:
  void triggerStop1();
  FrameSpeeds
  createFrameSpeedsMessage(const std::vector<double> &speeds_) const;
  void waitTillTFReady(const std::string &frame, const ros::Time &now,
                       const unsigned short int max_num_retries = 10) const;
  tf2::Vector3 getPose(const std::string &frame, const ros::Time &now) const;
  bool isWithinLimit(const double &speed) const;

private:
  static double speedFromTwoPoses(const tf2::Vector3 &a, const tf2::Vector3 &b,
                                  const double &t);

private:
  ros::NodeHandle nh_;
  const std::string reference_frame_;
  const std::vector<std::string> frames_to_observe_;
  ros::Publisher frame_speeds_pub_;
  ros::ServiceClient sto_client_;
  std::atomic_bool terminate_{false};

  tf2_ros::Buffer tf_buffer_;
  // Needed to receive tf2 transformations over the wire
  // For more infor see https://wiki.ros.org/tf2/Tutorials/
  tf2_ros::TransformListener tf_listener{tf_buffer_};
  double current_speed_limit_{DEFAULT_SPEED_LIMIT};

private:
  static constexpr double DEFAULT_SPEED_LIMIT{.25};
  static constexpr uint32_t DEFAULT_QUEUE_SIZE{10};
  static constexpr uint32_t WAITING_TIME_FOR_TRANSFORM_S{1};
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline double SpeedObserver::speedFromTwoPoses(const tf2::Vector3 &a,
                                               const tf2::Vector3 &b,
                                               const double &t) {
  double d = tf2::tf2Distance(a, b);
  return d / t;
}

inline bool SpeedObserver::isWithinLimit(const double &speed) const {
  return speed < current_speed_limit_;
}

// This method is only needed for the unittest
inline void SpeedObserver::terminateNow() {
  ROS_DEBUG("terminateNow");
  terminate_ = true;
}

} // namespace prbt_hardware_support

#endif // SPEED_OBSERVER_H
