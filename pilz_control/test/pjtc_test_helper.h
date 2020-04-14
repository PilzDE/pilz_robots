/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PILZ_CONTROL_PJTC_TEST_HELPER_H
#define PILZ_CONTROL_PJTC_TEST_HELPER_H

#include <chrono>
#include <functional>
#include <future>
#include <thread>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <std_srvs/Trigger.h>

#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#include "robot_mock.h"

namespace pilz_joint_trajectory_controller_test
{
static const std::string STOP_TRAJECTORY_DURATION_PARAMETER {"stop_trajectory_duration"};
static const std::string GOAL_TIME_TOLERANCE_PARAMETER {"constraints/goal_time"};
static const std::string JOINTS_PARAMETER {"joints"};

static constexpr double DEFAULT_GOAL_DURATION_SEC {1.0};
static constexpr double STOP_TRAJECTORY_DURATION_SEC {0.2};
static constexpr double GOAL_TIME_TOLERANCE_SEC {0.01};

static constexpr double TIME_SIMULATION_START_SEC {0.1};
static constexpr double DEFAULT_UPDATE_PERIOD_SEC {0.008};
static constexpr unsigned int SLEEP_TIME_MSEC {5};
static constexpr std::chrono::milliseconds HOLD_TIMEOUT {1000};
static constexpr std::chrono::milliseconds MOVEMENT_TIMEOUT {1000};

using UpdateFunc = std::function<void()>;
using GoalType = control_msgs::FollowJointTrajectoryGoal;

template<typename T>
bool isFutureReady(const std::future<T>& this_future)
{
  return this_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

static void startSimTime(const ros::Time& start_time = ros::Time(TIME_SIMULATION_START_SEC))
{
  ros::Time::setNow(start_time);
}

//! @brief Make progress in simulated ros::Time.
static void progressInTime(const ros::Duration& period)
{
  ros::Time current_time {ros::Time::now()};
  ros::Time::setNow(current_time + period);
}

//! @brief Set parameters required for the controller tests
static void setControllerParameters(const std::string& controller_ns)
{
  ros::NodeHandle controller_nh_ {controller_ns};

  std::vector<std::string> joint_names(JOINT_NAMES.begin(), JOINT_NAMES.end());

  controller_nh_.setParam(JOINTS_PARAMETER, joint_names);
  controller_nh_.setParam(STOP_TRAJECTORY_DURATION_PARAMETER, STOP_TRAJECTORY_DURATION_SEC);
  controller_nh_.setParam(GOAL_TIME_TOLERANCE_PARAMETER, GOAL_TIME_TOLERANCE_SEC);
}

static ros::Duration getGoalDuration(const control_msgs::FollowJointTrajectoryGoal& goal)
{
  return goal.trajectory.points.back().time_from_start;
}

/**
 * @brief Cycle through three joint positions in order to always get a movement even if the previous goal failed.
 */
static GoalType generateSimpleGoal(const ros::Duration& goal_duration = ros::Duration(DEFAULT_GOAL_DURATION_SEC))
{
  static unsigned int position_index {0};
  const std::vector<double> alternating_positions {0.5, -0.25, -0.5};
  position_index = (++position_index) % alternating_positions.size();

  std::vector<std::string> joint_names(JOINT_NAMES.begin(), JOINT_NAMES.end());

  GoalType goal;
  goal.trajectory.joint_names = joint_names;
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].time_from_start = goal_duration;
  goal.trajectory.points[0].positions = {alternating_positions[position_index], 0.0};

  return goal;
}

/**
 * @brief Either return true, when the condition is fulfilled or false, when the timeout has been reached
 * or ROS is shutdown.
 * @param is_condition_fulfilled Boolean function, which is expected to return true eventually.
 * @param timeout Timeout [ms] with respect to system time.
 * @param update_func Update function. If non-empty, the following is done periodically:
 * - Make progress in simulated ros::Time,
 * - Invoke update_func.
 */
static bool waitFor(const std::function<bool()>& is_condition_fulfilled,
                    const std::chrono::milliseconds& timeout,
                    const UpdateFunc& update_func = UpdateFunc())
{
  const std::chrono::system_clock::time_point start {std::chrono::system_clock::now()};
  do
  {
    if (is_condition_fulfilled())
    {
      return true;
    }
    if (std::chrono::system_clock::now() - start > timeout)
    {
      return false;
    }
    if (update_func)
    {
      progressInTime(ros::Duration(DEFAULT_UPDATE_PERIOD_SEC));
      update_func();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_MSEC));
  }
  while (ros::ok());
  return false;
}

template<class RobotDriver>
static bool updateUntilHoldMode(RobotDriver* robot_driver,
                                const std::future<bool>& hold_future,
                                const std::chrono::milliseconds hold_timeout = HOLD_TIMEOUT)
{
  return waitFor([&hold_future](){ return isFutureReady(hold_future); },
  hold_timeout, [robot_driver](){ robot_driver->update(); });
}

template<class RobotDriver>
static bool updateUntilRobotMotion(RobotDriver* robot_driver,
                                   const std::chrono::milliseconds movement_timeout = MOVEMENT_TIMEOUT)
{
  return waitFor([robot_driver](){ return robot_driver->isRobotMoving(); },
  movement_timeout, [robot_driver](){ robot_driver->update(); });
}

/**
 * @brief Perform init, start, unhold and update, such that controller is ready for executing.
 */
template <class RobotDriver>
static testing::AssertionResult performFullControllerStartup(RobotDriver* robot_driver)
{
  auto manager {robot_driver->getManager()};
  if (!manager->loadController())
  {
    return testing::AssertionFailure() << "Failed to initialize the controller.";
  }

  manager->startController();

  std::function<bool()> is_unhold_successful {[&manager]()
    {
      std_srvs::TriggerRequest req;
      std_srvs::TriggerResponse resp;
      manager->triggerUnHold(req, resp);
      return resp.success;
    }};

  // unhold will only be successful after hold is reached
  if (!waitFor(is_unhold_successful, HOLD_TIMEOUT, [robot_driver](){ robot_driver->update(); }))
  {
    return testing::AssertionFailure() << "Unholding the controller failed.";
  }

  return testing::AssertionSuccess();
}

}  // namespace pilz_joint_trajectory_controller

#endif  // PILZ_CONTROL_PJTC_TEST_HELPER_H
