/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#ifndef PRBT_HARDWARE_SUPPORT_ADAPTER_STO_H
#define PRBT_HARDWARE_SUPPORT_ADAPTER_STO_H

#include <atomic>
#include <mutex>
#include <thread>
#include <string>

#include <ros/time.h>
#include <ros/service_client.h>
#include <std_srvs/Trigger.h>

#include <prbt_hardware_support/service_client_factory.h>

namespace prbt_hardware_support
{

/**
 * @brief Stores the last reported STO stated and reacts to changes of the STO.
 * Reactions:
 *  - STO == false:   Perfrom Stop 1
 *  - STO == true:    unhold Controller + recover Drives
 *
 * @note PilzStoModbusAdapterNode::DURATION_BETWEEN_HOLD_AND_DISABLE_MS
 * specifies the time between holding the controller and halting the driver.
 * This allows the controller to perform a smooth stop before a driver disable
 * enables the breaks.
 *
 * @remark this class is templated for easier mocking. However for usability
 * it can be used by AdapterSto
 */
template <class T = ros::ServiceClient>
class AdapterStoTemplated
{
public:
  /**
   * @brief Connect to services.
   */
  AdapterStoTemplated(std::function<T(std::string)> create_service_client = ServiceClientFactory::create<std_srvs::Trigger>);

  /**
   * @brief Terminate run-loop.
   */
  virtual ~AdapterStoTemplated();

  /**
  * @brief Run a loop executing next_action_ everytime it has changed.
  *
  * next_action_ can change either asynchronously via updateSto()
  * or due to the autoUpdate() call after an execution finished.
  */
  void run();

  /**
   * @brief Call run() in separate thread.
   */
  void runAsync();

  /**
   * @brief Terminate the run-loop
   */
  void terminate();

public:
  static const std::string HOLD_SERVICE;
  static const std::string UNHOLD_SERVICE;
  static const std::string RECOVER_SERVICE;
  static const std::string HALT_SERVICE;

protected:
  /**
   * @brief Depending on the latest action and the \p STO determine the next action.
   */
  void updateSto(const bool sto);

private:
  enum Action
  {
    HALTING,
    RECOVERING,
    UNHOLDING,
    HOLDING
  };

  /**
   * @brief Execute the given action (service call)
   *
   * @return True if the respective service call returned true, false otherwise
   */
  bool execute(Action action);

  /**
   * @brief Perform an automatic update of the next action depending on \p finished_action
   */
  void autoUpdate(Action finished_action, bool exec_result);

private:
  //! ServiceClient attached to the controller <code>/hold</code> service
  T hold_srv_client_;

  //! ServiceClient attached to the controller <code>/unhold</code> service
  T unhold_srv_client_;

  //! ServiceClient attached to the driver <code>/recover</code> service
  T recover_srv_client_;

  //! ServiceClient attached to the driver <code>/halt</code> service
  T halt_srv_client_;

  //! termination flag for the run-loop
  std::atomic_bool terminate_{false};

  //! Thread for the run-loop
  std::thread run_thread_;

  //! The next action
  Action next_action_{Action::HALTING};

  //! Mutex protecting next_action_
  std::mutex action_mutex_;

private:
  /**
   * @brief Specifies the time between holding the controller and
   * disabling the driver. This allows the controller to perform a smooth stop
   * before a driver disable enables the breaks.
   */
  static constexpr int DURATION_BETWEEN_HOLD_AND_DISABLE_MS {200};
};

//! Simple typedef for class like usage
typedef AdapterStoTemplated<> AdapterSto;

template <class T>
const std::string AdapterStoTemplated<T>::HOLD_SERVICE{"manipulator_joint_trajectory_controller/hold"};

template <class T>
const std::string AdapterStoTemplated<T>::UNHOLD_SERVICE{"manipulator_joint_trajectory_controller/unhold"};

template <class T>
const std::string AdapterStoTemplated<T>::RECOVER_SERVICE{"driver/recover"};

template <class T>
const std::string AdapterStoTemplated<T>::HALT_SERVICE{"driver/halt"};

template <class T>
AdapterStoTemplated<T>::AdapterStoTemplated(std::function<T(std::string)> create_service_client)
    : hold_srv_client_(create_service_client(HOLD_SERVICE)),
      unhold_srv_client_(create_service_client(UNHOLD_SERVICE)),
      recover_srv_client_(create_service_client(RECOVER_SERVICE)),
      halt_srv_client_(create_service_client(HALT_SERVICE))
{
}

template <class T>
void AdapterStoTemplated<T>::run()
{
  Action current_action{Action::HALTING};
  bool execute_action{false};

  ros::Rate rate(10);
  while (!terminate_)
  {
    {
      std::lock_guard<std::mutex> lock(action_mutex_);
      execute_action = execute_action || (next_action_ != current_action);
      if (execute_action)
      {
        current_action = next_action_;
      }
    }

    if (execute_action)
    {
      bool exec_result = execute(current_action);
      execute_action = !exec_result;
      autoUpdate(current_action, exec_result);
    }

    rate.sleep();
  }
}

template <class T>
AdapterStoTemplated<T>::~AdapterStoTemplated()
{
  terminate();
}

template <class T>
void AdapterStoTemplated<T>::runAsync()
{
  run_thread_ = std::thread{[this] { this->run(); }};
}

template <class T>
void AdapterStoTemplated<T>::updateSto(const bool STO)
{
  std::lock_guard<std::mutex> lock(action_mutex_);

  if (STO)
  {
    if (next_action_ == Action::HALTING)
    {
      next_action_ = Action::RECOVERING;
    }
    else if (next_action_ == Action::HOLDING)
    {
      next_action_ = Action::UNHOLDING;
    }
  }
  else
  {
    if (next_action_ == Action::RECOVERING)
    {
      next_action_ = Action::HALTING;
    }
    else if (next_action_ == Action::UNHOLDING)
    {
      next_action_ = Action::HOLDING;
    }
  }
}

template <class T>
void AdapterStoTemplated<T>::terminate()
{
  terminate_ = true;
  if (run_thread_.joinable())
  {
    run_thread_.join();
  }
}

template <class T>
void AdapterStoTemplated<T>::autoUpdate(Action finished_action, bool exec_result)
{
  std::lock_guard<std::mutex> lock(action_mutex_);

  if (next_action_ == finished_action)
  {
    switch (finished_action)
    {
    case Action::RECOVERING:
      if (exec_result)
      {
        next_action_ = Action::UNHOLDING;
      }
      break;
    case Action::HOLDING:
      next_action_ = Action::HALTING;
      break;
    default:
      break;
    }
  }
}

template <class T>
bool AdapterStoTemplated<T>::execute(Action action)
{
  bool success{false};

  switch (action)
  {
  case Action::RECOVERING:
  {
    std_srvs::Trigger trigger;
    ROS_DEBUG_STREAM("Calling Recover (Service: " << recover_srv_client_.getService() << ")");
    success = recover_srv_client_.call(trigger);

    if (!success)
    {
      ROS_ERROR_STREAM("No success calling Recover (Service: " << recover_srv_client_.getService() << ")");
    }
    break;
  }
  case Action::UNHOLDING:
  {
    std_srvs::Trigger trigger;
    ROS_DEBUG_STREAM("Calling Unhold (Service: " << unhold_srv_client_.getService() << ")");
    success = unhold_srv_client_.call(trigger);

    if (!success)
    {
      ROS_ERROR_STREAM("No success calling Unhold (Service: " << unhold_srv_client_.getService() << ")");
    }
    break;
  }
  case Action::HOLDING:
  {
    std_srvs::Trigger trigger;
    ROS_DEBUG_STREAM("Calling Hold on controller (Service: " << hold_srv_client_.getService() << ")");
    success = hold_srv_client_.call(trigger);

    if (!success)
    {
      ROS_ERROR_STREAM("No success calling Hold on controller (Service: " << hold_srv_client_.getService() << ")");
      break;
    }
    ros::Duration(DURATION_BETWEEN_HOLD_AND_DISABLE_MS / 1000.0).sleep(); // wait until hold traj is finished
    break;
  }
  case Action::HALTING:
  {
    std_srvs::Trigger trigger;
    ROS_DEBUG_STREAM("Calling Halt on driver (Service: " << halt_srv_client_.getService() << ")");
    success = halt_srv_client_.call(trigger);

    if (!success)
    {
      ROS_ERROR_STREAM("No success calling Halt on driver (Service: " << halt_srv_client_.getService() << ")");
    }
    break;
  }
  default:
    break;
  }

  return success;
}

}
#endif // PRBT_HARDWARE_SUPPORT_ADAPTER_STO_H
