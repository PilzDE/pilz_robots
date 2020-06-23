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

void JointStatePublisherMock::startPublishingAsync(const double& joint1_start_position)
{
  stop_flag_ = false;
  joint1_position_ = joint1_start_position;
  publisher_thread_ = std::thread(&JointStatePublisherMock::run, this);
}

void JointStatePublisherMock::setJoint1Velocity(const double& vel)
{
  go_home_flag_ = false;
  std::lock_guard<std::mutex> lock(next_msg_mutex_);
  joint1_velocity_ = vel;
}

void JointStatePublisherMock::goHome()
{
  go_home_flag_ = true;
}

void JointStatePublisherMock::stopPublishing()
{
  if (publisher_thread_.joinable())
  {
    stop_flag_ = true;
    publisher_thread_.join();
  }
}

sensor_msgs::JointStateConstPtr JointStatePublisherMock::getNextMessage()
{
  std::lock_guard<std::mutex> lock(next_msg_mutex_);
  return boost::make_shared<JointState>(next_msg_);
}

void JointStatePublisherMock::run()
{
  next_time_stamp_ = ros::Time::now();
  createNextMessage();

  ros::Rate rate{ JOINT_STATES_RATE };
  while (ros::ok() && !stop_flag_)
  {
    next_time_stamp_ = next_time_stamp_ + rate.expectedCycleTime();

    std::unique_lock<std::mutex> lock(next_msg_mutex_);
    publish();
    updateJoint1Position();
    updateNextMessage();
    lock.unlock();

    rate.sleep();
  }
}

void JointStatePublisherMock::createNextMessage()
{
  next_msg_.name = joint_names_;
  next_msg_.header.stamp = next_time_stamp_;
  next_msg_.position.resize(joint_names_.size(), 0.0);
  next_msg_.velocity.resize(joint_names_.size(), 0.0);
  next_msg_.effort.resize(joint_names_.size(), 0.0);

  next_msg_.position.at(0) = joint1_position_;
}

void JointStatePublisherMock::publish()
{
  pub_.publish(next_msg_);
}

void JointStatePublisherMock::updateNextMessage()
{
  next_msg_.header.stamp = next_time_stamp_;
  next_msg_.position.at(0) = joint1_position_;
}

void JointStatePublisherMock::updateJoint1Position()
{
  double delta = joint1_velocity_ / static_cast<double>(JOINT_STATES_RATE);

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
