/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
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

#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gmock/gmock.h>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <pilz_utils/get_param.h>

#include <pilz_testutils/joint_state_publisher_mock.h>

namespace pilz_testutils
{
static const std::string JOINT_NAMES_PARAMETER{ "controller_joint_names" };
static const std::string JOINT_STATES_TOPIC_NAME{ "joint_states" };

static constexpr unsigned int JOINT_STATES_RATE{ 25 };
static constexpr unsigned int JOINT_STATES_QUEUE_SIZE{ 25 };

static constexpr double JOINT1_ABSOLUTE_POSITION_LIMIT{ 3.0 };

inline int signum(const double& value)
{
  return (value > 0) - (0 > value);
}

JointStatePublisherMock::JointStatePublisherMock()
{
  joint_names_ = pilz_utils::getParam<std::vector<std::string>>(nh_, JOINT_NAMES_PARAMETER);
  assert(!joint_names_.empty());

  pub_ = nh_.advertise<JointState>(nh_.getNamespace() + "/" + JOINT_STATES_TOPIC_NAME, JOINT_STATES_QUEUE_SIZE);
}

void JointStatePublisherMock::startAsync()
{
  stop_flag_ = false;
  publisher_thread_ = std::thread(&JointStatePublisherMock::run, this);
}

void JointStatePublisherMock::setVelocity(const double& joint1_velocity)
{
  go_home_flag_ = false;
  std::lock_guard<std::mutex> lock(joint1_velocity_mutex_);
  joint1_velocity_ = joint1_velocity;
}

void JointStatePublisherMock::goHome()
{
  go_home_flag_ = true;
}

void JointStatePublisherMock::stop()
{
  if (publisher_thread_.joinable())
  {
    stop_flag_ = true;
    publisher_thread_.join();
  }
}

void JointStatePublisherMock::setDegenerateTimeMode()
{
  degenerate_time_flag_ = true;
}

sensor_msgs::JointStateConstPtr JointStatePublisherMock::getNextMessage()
{
  JointState msg;
  createMessage(msg);
  return boost::make_shared<JointState>(msg);
}

void JointStatePublisherMock::run()
{
  ros::Rate rate{ JOINT_STATES_RATE };
  while (ros::ok() && !stop_flag_)
  {
    JointState msg;
    createMessage(msg);
    pub_.publish(msg);
    if (degenerate_time_flag_)
    {
      pub_.publish(msg);
    }
    updateJoint1Position();
    rate.sleep();
  }
}

void JointStatePublisherMock::createMessage(JointState& msg)
{
  msg.name = joint_names_;
  msg.header.stamp = ros::Time::now();
  msg.position.resize(joint_names_.size(), 0.0);
  msg.velocity.resize(joint_names_.size(), 0.0);
  msg.effort.resize(joint_names_.size(), 0.0);

  std::lock_guard<std::mutex> lock(joint1_position_mutex_);
  msg.position.at(0) = joint1_position_;
}

void JointStatePublisherMock::updateJoint1Position()
{
  std::unique_lock<std::mutex> velocity_lock(joint1_velocity_mutex_);
  double delta = joint1_velocity_ / static_cast<double>(JOINT_STATES_RATE);
  velocity_lock.unlock();

  std::lock_guard<std::mutex> position_lock(joint1_position_mutex_);
  if (go_home_flag_)
  {
    if (std::abs(delta) > std::abs(joint1_position_))
    {
      joint1_position_ = 0.0;
      return;
    }
    double position_sign = static_cast<double>(signum(joint1_position_));
    joint1_position_ -= position_sign * delta;
    return;
  }

  static double sign = 1.0;
  if (std::abs(joint1_position_) >= JOINT1_ABSOLUTE_POSITION_LIMIT)
  {
    sign *= -1.0;
  }
  joint1_position_ += sign * delta;
}

}  // namespace pilz_testutils
