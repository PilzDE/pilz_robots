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

#ifndef PRBT_HARDWARE_SUPPORT_JOINT_STATES_PUBLISHER_MOCK_H
#define PRBT_HARDWARE_SUPPORT_JOINT_STATES_PUBLISHER_MOCK_H

#include <atomic>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace prbt_hardware_support
{

/**
 * @brief Asynchronously publishes predefined messages on the /joint_states topic with rate ~100Hz.
 */
class JointStatesPublisherMock
{
public:
  JointStatesPublisherMock();

  ~JointStatesPublisherMock();

  /**
   * @brief Start a new thread publishing joint states.
   *
   * @param move If true, a movement is simulated, otherwise the positions do not change.
   */
  void startAsync(bool move = false);

  void terminate();

  /**
   * @brief Obtain the message that is published next.
   */
  sensor_msgs::JointStateConstPtr getNextMessage();

private:
  void start(bool positions_fixed);

private:
  ros::NodeHandle nh_;
  ros::Publisher joint_states_pub_;
  std::thread thread_;
  std::atomic_bool terminate_;
  std::mutex msg_mutex_;
  sensor_msgs::JointState msg_;
};

} // namespace prbt_hardware_support

#endif
