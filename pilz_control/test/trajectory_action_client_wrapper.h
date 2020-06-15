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

#ifndef PILZ_CONTROL_TRAJECTORY_ACTION_CLIENT_WRAPPER_H
#define PILZ_CONTROL_TRAJECTORY_ACTION_CLIENT_WRAPPER_H

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include "pjtc_test_helper.h"

namespace pilz_joint_trajectory_controller_test
{
static constexpr unsigned int WAIT_FOR_ACTION_SERVER_TIMEOUT_MSEC{ 5000 };
static constexpr double WAIT_FOR_ACTION_RESULT_TIMEOUT_SEC{ 5.0 };

/**
 * @brief Provides convenient functions for using a FollowJointTrajectory action interface.
 * @note Inteded for usage with simulated ros::Time.
 */
class TrajectoryActionClientWrapper
{
private:
  using Action = control_msgs::FollowJointTrajectoryAction;
  using ActionClient = actionlib::SimpleActionClient<Action>;
  using Goal = control_msgs::FollowJointTrajectoryGoal;
  using ResultConstPtr = control_msgs::FollowJointTrajectoryResultConstPtr;

public:
  TrajectoryActionClientWrapper(const std::string& action_name);

  //! @brief Only pass through to action client.
  void sendGoal(const Goal& goal);

  //! @brief Only pass through to action client.
  void cancelGoal();

  //! @brief Only pass through from action client.
  ResultConstPtr getResult() const;

  //! @brief Wait independently of ros::Time opposed to actionlib function waitForServer().
  bool waitForActionServer(
      const std::chrono::milliseconds& timeout = std::chrono::milliseconds(WAIT_FOR_ACTION_SERVER_TIMEOUT_MSEC));

  /**
   * @brief Wait for action result while making periodic progress in simulated ros::Time.
   * @param update_func Invoked periodically, if non-empty.
   * @param timeout Timeout with respect to ros::Time.
   */
  bool waitForActionResult(const UpdateFunc& update_func = UpdateFunc(),
                           const ros::Duration& timeout = ros::Duration(WAIT_FOR_ACTION_RESULT_TIMEOUT_SEC));

private:
  std::unique_ptr<ActionClient> action_client_;
};

inline TrajectoryActionClientWrapper::TrajectoryActionClientWrapper(const std::string& action_name)
{
  action_client_.reset(new ActionClient(action_name, true));
}

inline void TrajectoryActionClientWrapper::sendGoal(const Goal& goal)
{
  action_client_->sendGoal(goal);
}

inline void TrajectoryActionClientWrapper::cancelGoal()
{
  action_client_->cancelGoal();
}

inline TrajectoryActionClientWrapper::ResultConstPtr TrajectoryActionClientWrapper::getResult() const
{
  return action_client_->getResult();
}

inline bool TrajectoryActionClientWrapper::waitForActionServer(const std::chrono::milliseconds& timeout)
{
  return waitFor([this]() { return action_client_->isServerConnected(); }, timeout);
}

bool TrajectoryActionClientWrapper::waitForActionResult(const UpdateFunc& update_func, const ros::Duration& timeout)
{
  std::future<bool> wait_for_result_future =
      std::async(std::launch::async, [this, &timeout]() { return action_client_->waitForResult(timeout); });
  while (!isFutureReady(wait_for_result_future))
  {
    progressInTime(ros::Duration(DEFAULT_UPDATE_PERIOD_SEC));
    if (update_func)
    {
      update_func();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_MSEC));
  }
  return wait_for_result_future.get();
}

}  // namespace pilz_joint_trajectory_controller_test

#endif  // PILZ_CONTROL_TRAJECTORY_ACTION_CLIENT_WRAPPER_H
