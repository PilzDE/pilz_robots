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
#include <mutex>
#include <thread>
#include <chrono>

#include <sensor_msgs/JointState.h>

#include <prbt_hardware_support/joint_states_publisher_mock.h>

namespace prbt_hardware_support
{

static const std::string JOINT_STATES_TOPIC_NAME{"/prbt/joint_states"};
static constexpr unsigned int JOINT_STATES_TOPIC_QUEUE_SIZE{1};

JointStatesPublisherMock::JointStatesPublisherMock()
{
  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>(JOINT_STATES_TOPIC_NAME, JOINT_STATES_TOPIC_QUEUE_SIZE);
  msg_.name = {"joint1", "joint2"};
  msg_.position = {0.1, -0.11};
}

JointStatesPublisherMock::~JointStatesPublisherMock()
{
  terminate();
}

void JointStatesPublisherMock::startAsync(bool move)
{
  terminate_ = false;
  thread_ = std::thread{ [this, move]{ this->start(move); } };
}

void JointStatesPublisherMock::terminate()
{
  terminate_ = true;
  if (thread_.joinable())
  {
    thread_.join();
  }
}

sensor_msgs::JointStateConstPtr JointStatesPublisherMock::getNextMessage()
{
  std::lock_guard<std::mutex> lock(msg_mutex_);
  sensor_msgs::JointStateConstPtr msg(new sensor_msgs::JointState(msg_));
  return msg;
}

void JointStatesPublisherMock::start(bool move)
{
  while (!terminate_)
  {
    {
      std::lock_guard<std::mutex> lock(msg_mutex_);
      sensor_msgs::JointStateConstPtr msg(new sensor_msgs::JointState(msg_));
      joint_states_pub_.publish(msg);
      if (move)
      {
        // change positions; reaches limit after > 100 seconds
        msg_.position.at(0) = std::min(1000.0, msg_.position.at(0)+0.1);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

} // namespace prbt_hardware_support
