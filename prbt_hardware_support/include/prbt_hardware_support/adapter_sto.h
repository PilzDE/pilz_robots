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

#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_VECTOR_SIZE 30 //or whatever you need
#define BOOST_MPL_LIMIT_MAP_SIZE 30 //or whatever you need

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <ros/service_client.h>
#include <std_srvs/Trigger.h>

#include <prbt_hardware_support/service_client_factory.h>
#include <prbt_hardware_support/sto_state_machine.h>

namespace prbt_hardware_support
{

/**
 * @brief Performs service calls for Stop1 and the respective reversal, that is enabling the manipulator. Incoming
 * updates of the STO state are handled asynchronously.
 *
 * In order to handle the asynchrony of events, a state machine is used to represent the current state. The state machine
 * manages a task queue for the currently required service call.
 *
 * General behaviour:
 *  - STO == false:   perfom Stop1 (hold controller + halt drives)
 *  - STO == true:    recover drives + unhold controller
 *
 * @note Unhold the controller is skipped if STO changes during recover.
 * This avoids the superfluous execution of a hold trajectory, which would result in an overlong stopping time.
 *
 * @note If a service call fails, the execution is always continued in order to make a Stop1 or a recover-retry possible.
 *
 * @remark this class is templated for easier mocking. However for usability
 * it can be used by AdapterSto
 */
template <class T = ros::ServiceClient>
class AdapterStoTemplated
{
public:
  /**
   * @brief Create required service clients and state machine; start worker-thread and state machine.
   *
   * @param create_service_client Returns a service client. Can be used to return a mock class in tests.
   */
  AdapterStoTemplated(const std::function<T(std::string)> &create_service_client = ServiceClientFactory::create<std_srvs::Trigger>);

  /**
   * @brief Stop state machine and terminate worker-thread.
   */
  ~AdapterStoTemplated();

  /**
   * @brief This is called everytime an updated sto value is obtained.
   *
   * Process sto_updated event and notify worker-thread in case it is waiting for new required tasks.
   *
   * @note
   * Access to the state machine is protected for thread-safety.
   *
   * @param sto The updated sto value.
   */
  void updateSto(const bool sto);

  /**
   * @brief Call the service triggering recover of the drives.
   */
  void call_recover();

  /**
   * @brief Call the service triggering halt of the drives.
   */
  void call_halt();

  /**
   * @brief Call the service triggering hold of the controller. Wait until execution of the hold trajectory finished.
   */
  void call_hold();

  /**
   * @brief Call the service triggering unhold of the controller.
   */
  void call_unhold();

public:
  static const std::string HOLD_SERVICE;
  static const std::string UNHOLD_SERVICE;
  static const std::string RECOVER_SERVICE;
  static const std::string HALT_SERVICE;
  static const std::string IS_EXECUTING_SERVICE;

private:

  /**
   * @brief This is executed in the worker-thread and allows asynchronous handling of sto updates.
   *
   * Wait for notification if the task queue of the state machine is empty. Once a task is present, execute it and signal
   * its completion.
   *
   * @note
   * Access to the state machine is protected for thread-safety. It is assumed that task execution does not access
   * the state machine, whereas the completion signalling does.
   */
  void workerThreadFun();

private:
  //! State machine
  std::unique_ptr<StoStateMachine> state_machine_;

  //! Flag indicating if the worker-thread should terminate
  std::atomic_bool terminate_{false};

  //! Mutex for protecting access to the state machine
  std::mutex sm_mutex_;

  //! Condition variable for notifying a waiting worker-thread
  std::condition_variable worker_cv_;

  //! Worker-thread
  std::thread worker_thread_;

  //! ServiceClient attached to the controller <code>/hold</code> service
  T hold_srv_client_;

  //! ServiceClient attached to the controller <code>/unhold</code> service
  T unhold_srv_client_;

  //! ServiceClient attached to the driver <code>/recover</code> service
  T recover_srv_client_;

  //! ServiceClient attached to the driver <code>/halt</code> service
  T halt_srv_client_;

  //! ServiceClient attached to the controller <code>/is_executing</code> service
  T is_executing_srv_client_;

  //! Rate for calling the is_executing service of the controller during hold
  static constexpr int WAIT_FOR_IS_EXECUTING_RATE{100};

  /**
   * @brief Specifies when to abort waiting for the execution of a hold trajectory. This should not be smaller
   * than the duration of the hold trajectory, such that in any case the driver halt is not called too early.
   */
  static constexpr double WAIT_FOR_IS_EXECUTING_TIMEOUT_S{0.2};
};

//! typedef for simple usage
using AdapterSto = AdapterStoTemplated<>;

template <class T>
const std::string AdapterStoTemplated<T>::HOLD_SERVICE{"manipulator_joint_trajectory_controller/hold"};

template <class T>
const std::string AdapterStoTemplated<T>::UNHOLD_SERVICE{"manipulator_joint_trajectory_controller/unhold"};

template <class T>
const std::string AdapterStoTemplated<T>::RECOVER_SERVICE{"driver/recover"};

template <class T>
const std::string AdapterStoTemplated<T>::HALT_SERVICE{"driver/halt"};

template <class T>
const std::string AdapterStoTemplated<T>::IS_EXECUTING_SERVICE{"manipulator_joint_trajectory_controller/is_executing"};

template <class T>
AdapterStoTemplated<T>::AdapterStoTemplated(const std::function<T(std::string)> &create_service_client)
    : hold_srv_client_(create_service_client(HOLD_SERVICE)),
      unhold_srv_client_(create_service_client(UNHOLD_SERVICE)),
      recover_srv_client_(create_service_client(RECOVER_SERVICE)),
      halt_srv_client_(create_service_client(HALT_SERVICE)),
      is_executing_srv_client_(create_service_client(IS_EXECUTING_SERVICE))
{
  state_machine_ = std::unique_ptr<StoStateMachine>(new StoStateMachine(
    std::bind(&AdapterStoTemplated<T>::call_recover, this),
    std::bind(&AdapterStoTemplated<T>::call_halt, this),
    std::bind(&AdapterStoTemplated<T>::call_hold, this),
    std::bind(&AdapterStoTemplated<T>::call_unhold, this)));

  ROS_DEBUG("Start worker-thread");
  worker_thread_ = std::thread(&AdapterStoTemplated<T>::workerThreadFun, this);

  ROS_DEBUG("Start state machine");
  state_machine_->start();
}

template <class T>
AdapterStoTemplated<T>::~AdapterStoTemplated()
{
  ROS_DEBUG("Stop state machine");
  state_machine_->stop();

  if (worker_thread_.joinable())
  {
    ROS_DEBUG("Join worker thread");
    terminate_ = true;
    worker_cv_.notify_one();
    worker_thread_.join();
  }
}

template <class T>
void AdapterStoTemplated<T>::updateSto(const bool sto)
{
  ROS_DEBUG("updateSto called");
  {
    std::lock_guard<std::mutex> lock(sm_mutex_);
    ROS_DEBUG("trigger sto_updated");
    state_machine_->process_event(typename StoStateMachine::sto_updated(sto));
  }
  worker_cv_.notify_one();
}
template <class T>
void AdapterStoTemplated<T>::workerThreadFun()
{
  std::unique_lock<std::mutex> sm_lock(sm_mutex_);
  while (!terminate_)
  {
    ROS_DEBUG("Wait for task or termination");
    worker_cv_.wait(sm_lock, [this]() { return (!this->state_machine_->task_queue_.empty() || this->terminate_); });
    if (terminate_)
    {
      break;
    }

    ROS_DEBUG("Obtain task");
    AsyncStoTask task = state_machine_->task_queue_.front();
    state_machine_->task_queue_.pop();

    sm_lock.unlock();
    ROS_DEBUG("Execute task");
    task.execute();

    sm_lock.lock();
    ROS_DEBUG("trigger completion event");
    task.signalCompletion();
  }
}

template <class T>
void AdapterStoTemplated<T>::call_recover()
{
  std_srvs::Trigger recover_trigger;
  ROS_DEBUG_STREAM("Calling Recover (Service: " << recover_srv_client_.getService() << ")");
  bool recover_success = recover_srv_client_.call(recover_trigger);
  ROS_DEBUG_STREAM("Finished Recover (Service: " << recover_srv_client_.getService() << ")");

  if (!recover_success)
  {
    ROS_ERROR_STREAM("No success calling Recover (Service: " << recover_srv_client_.getService() << ")");
  }
}

template <class T>
void AdapterStoTemplated<T>::call_unhold()
{
  std_srvs::Trigger unhold_trigger;
  ROS_DEBUG_STREAM("Calling Unhold (Service: " << unhold_srv_client_.getService() << ")");
  bool unhold_success = unhold_srv_client_.call(unhold_trigger);

  if (!unhold_success)
  {
    ROS_ERROR_STREAM("No success calling Unhold (Service: " << unhold_srv_client_.getService() << ")");
  }
}

template <class T>
void AdapterStoTemplated<T>::call_hold()
{
  std_srvs::Trigger hold_trigger;
  ROS_DEBUG_STREAM("Calling Hold on controller (Service: " << hold_srv_client_.getService() << ")");
  bool hold_success = hold_srv_client_.call(hold_trigger);

  if (!hold_success)
  {
    ROS_ERROR_STREAM("No success calling Hold on controller (Service: " << hold_srv_client_.getService() << ")");
  }

  bool is_executing{false};
  ros::Rate rate{WAIT_FOR_IS_EXECUTING_RATE};
  ros::Time start_waiting{ros::Time::now()};

  // wait for execution of hold trajectory to begin
  while (!is_executing && !terminate_)
  {
    if (ros::Time::now() - start_waiting > ros::Duration(WAIT_FOR_IS_EXECUTING_TIMEOUT_S))
    {
      ROS_ERROR("No hold trajectory executed.");
      return;
    }

    std_srvs::Trigger is_executing_trigger;
    bool is_executing_success = is_executing_srv_client_.call(is_executing_trigger);
    if (!is_executing_success)
    {
      ROS_ERROR_STREAM("No success calling service " << is_executing_srv_client_.getService());
    }
    else
    {
      is_executing = is_executing_trigger.response.success;
    }

    rate.sleep();
  }

  start_waiting = ros::Time::now();
  // wait for execution of hold trajectory to end
  while (is_executing && !terminate_)
  {
    std_srvs::Trigger is_executing_trigger;
    bool is_executing_success = is_executing_srv_client_.call(is_executing_trigger);
    if (!is_executing_success)
    {
      ROS_ERROR_STREAM("No success calling service " << is_executing_srv_client_.getService());
      // In case the service call fails, make sure to return eventually
      if (ros::Time::now() - start_waiting > ros::Duration(WAIT_FOR_IS_EXECUTING_TIMEOUT_S))
      {
        return;
      }
    }
    else
    {
      is_executing = is_executing_trigger.response.success;
    }

    rate.sleep();
  }
}

template <class T>
void AdapterStoTemplated<T>::call_halt()
{
  std_srvs::Trigger halt_trigger;
  ROS_DEBUG_STREAM("Calling Halt on driver (Service: " << halt_srv_client_.getService() << ")");
  bool halt_success = halt_srv_client_.call(halt_trigger);

  if (!halt_success)
  {
    ROS_ERROR_STREAM("No success calling Halt on driver (Service: " << halt_srv_client_.getService() << ")");
  }
}

} // namespace prbt_hardware_support

#endif // PRBT_HARDWARE_SUPPORT_ADAPTER_STO_H
