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
#include <map>
#include <vector>

#include <ros/node_handle.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <prbt_hardware_support/FrameSpeeds.h>
#include <prbt_hardware_support/SetSpeedLimit.h>

namespace prbt_hardware_support
{
//! Allowed number of missed calculations. If it is exceeded a Stop1 is triggered.
static constexpr unsigned int DEFAULT_ALLOWED_MISSED_CALCULATIONS{3};

class SpeedObserver
{
public:
  /**
   * @brief Create an observer to observe the speed of a list of frames in reference to a reference frame.
   * @param nh NodeHandle to handle node
   * @param reference_frame Reference frame for all transformations
   * @param frames_to_observe List of frames to observer
   */
  SpeedObserver(ros::NodeHandle& nh, std::string& reference_frame, std::vector<std::string>& frames_to_observe);

public:
  /**
   * @brief Starts the observation cycle. The function blocks until ros shuts down.
   * @param frequency [Hz] Will check all frame once per cycle
   * @param allowed_missed_calculations Number of iterations that can have outdated tf data before a Stop1 is triggered.
   */
  void startObserving(const double frequency, const unsigned int allowed_missed_calculations = DEFAULT_ALLOWED_MISSED_CALCULATIONS);

  /**
   * @brief Callback for service to set the currently active speed limit.
   * @param req Service request
   * @param res Service response
   * @return True if the limit was set succesfully
   */
  bool setSpeedLimitCb(SetSpeedLimit::Request& req, SetSpeedLimit::Response& res);

  /**
   * @brief This method will terminate the observation cycle started by `startObserving()`.
   */
  void terminateNow();

private:
  /**
   * @brief Used to trigger stop of the robot when speed limit is exceeded.
   */
  void triggerStop1();

  /**
   * @brief Creates a message to be sent to the 'frame_speeds' topic.
   * @param speeds Vector containing one speed per observed frame
   * @return The message
   */
  FrameSpeeds createFrameSpeedsMessage(const std::map<std::string, double>& speeds) const;

  /**
   * @brief Helper method waiting until TF transformation is available.
   * @note Refernce frame is always `reference_frame_`
   * @param frame Which fram should be transformed
   * @param time At what time is the transfomration needed
   * @param max_num_retries How many times should the method wait for `WAITING_TIME_FOR_TRANSFORM_S` seconds
   */
  void waitUntillCanTransform(const std::string& frame, const ros::Time& time,
                              const unsigned short int max_num_retries = 10) const;

  /**
   * @brief Using tf to get the latest Pose of a frame as `tf::Vector3`.
   * @note Reference frame is always `reference_frame_`
   * @param frame Which frame should be transformed
   * @return The pose as Vector and the time stamp
   */
  std::pair<tf2::Vector3, ros::Time> getLatestPose(const std::string& frame) const;

  /**
   * @brief Check if a speed value is within the currently set limit.
   * @note Needed for unittest
   * @param speed The speed to check
   * @return True iff speed is within limit
   */
  bool isWithinLimit(const double& speed) const;

private:
  /**
   * @brief Calculate the minimal speed that a trajectory between two poses had if the motion was performed within a
   * given time.
   * @param a Pose at the beginning of the trajectory
   * @param b Pose at the end of the trajectory
   * @param t The time the motion took
   * @return The minimal speed for the trajectory
   */
  static double speedFromTwoPoses(const tf2::Vector3& a, const tf2::Vector3& b, const double& t);

private:
  ros::NodeHandle nh_;
  //! Publisher for frame speed message
  ros::Publisher frame_speeds_pub_;
  //! Client for sto service
  ros::ServiceClient sto_client_;
  //! Needed to receive tf2 transformations over the wire, see https://wiki.ros.org/tf2/Tutorials/
  tf2_ros::Buffer tf_buffer_;
  //! Listing to TF Transforms
  tf2_ros::TransformListener tf_listener{ tf_buffer_ };

  //! Reference frame for all speeds
  const std::string reference_frame_;
  //! All frames to observe
  const std::vector<std::string> frames_to_observe_;
  //! Helper variable to stop observation cycle
  std::atomic_bool terminate_{ false };
  //! Currently active speed limit
  double current_speed_limit_{ DEFAULT_SPEED_LIMIT };
  //! Map to store the number of successive missed frame transform calculations
  std::map<std::string, unsigned int> missed_calculations_;

private:
  //! Speed limit to be set at launch
  static constexpr double DEFAULT_SPEED_LIMIT{ .25 };
  //! Default queue size for publisher
  static constexpr uint32_t DEFAULT_QUEUE_SIZE{ 10 };
  //! Waiting time for `waitUntillCanTransform()`
  static constexpr double WAITING_TIME_FOR_TRANSFORM_S{ 0.1 };
  //! Epsilon prevents computation of speed for small time intervals
  static constexpr double TIME_INTERVAL_EPSILON_S{ 1e-9 };
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline double SpeedObserver::speedFromTwoPoses(const tf2::Vector3& a, const tf2::Vector3& b, const double& t)
{
  ROS_ASSERT(t != 0);
  double d = tf2::tf2Distance(a, b);
  return d / t;
}

inline bool SpeedObserver::isWithinLimit(const double& speed) const
{
  return speed < current_speed_limit_;
}

inline void SpeedObserver::terminateNow()
{
  ROS_DEBUG("terminateNow");
  terminate_ = true;
}

}  // namespace prbt_hardware_support

#endif  // SPEED_OBSERVER_H
