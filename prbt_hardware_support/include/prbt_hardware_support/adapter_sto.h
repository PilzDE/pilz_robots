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
#include <condition_variable>
#include <mutex>
#include <thread>
#include <string>
#include <queue>

#include <boost/variant.hpp>

#include <ros/ros.h>
#include <ros/service_client.h>
#include <std_srvs/Trigger.h>

#include <prbt_hardware_support/service_client_factory.h>
#include <prbt_hardware_support/sto_state_machine.h>

namespace prbt_hardware_support
{

/**
 * @brief Stores the last reported STO stated and reacts to changes of the STO.
 * Reactions:
 *  - STO == false:   perfom Stop 1
 *  - STO == true:    recover drives + unhold controller
 *
 * @note Unhold the controller is skipped if STO changes during recover.
 * This avoids the superfluous execution of a hold trajectory, which would result in a overlong stopping time.
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
  typedef StoStateMachine<AdapterStoTemplated<T>> StateMachine;
  typedef boost::variant<typename StateMachine::recover_done,
                         typename StateMachine::halt_done,
                         typename StateMachine::hold_done,
                         typename StateMachine::unhold_done> CompletionEvent;

  class Task
  {
  public:
    Task(std::function<void(AdapterStoTemplated<T>*)> &op, CompletionEvent &evt)
        : operation_(op),
          completion_event_(evt)
    {
    }

    CompletionEvent operator()(AdapterStoTemplated<T> *adapter)
    {
      operation_(adapter);
      return completion_event_;
    }

  private:
    std::function<void(AdapterStoTemplated<T>*)> operation_;

    CompletionEvent completion_event_;
  };

  typedef std::queue<Task> TaskQueue;

public:
  AdapterStoTemplated(std::function<T(std::string)> create_service_client = ServiceClientFactory::create<std_srvs::Trigger>);

  ~AdapterStoTemplated();

  void updateSto(bool sto);

  void call_recover();

  void call_halt();

  void call_hold();

  void call_unhold();

public:
  static const std::string HOLD_SERVICE;
  static const std::string UNHOLD_SERVICE;
  static const std::string RECOVER_SERVICE;
  static const std::string HALT_SERVICE;
  static const std::string IS_EXECUTING_SERVICE;

private:
  void workerThreadFun();

private:
  //! Task queue consists of single task
  std::shared_ptr<TaskQueue> task_queue_{std::make_shared<TaskQueue>()};

  //! State machine
  StateMachine state_machine_;

  std::atomic_bool terminate_{false};
  std::mutex sm_mutex_;
  std::mutex task_mutex_;
  std::condition_variable worker_cv_;
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

  /**
   * @brief Specifies the time between holding the controller and
   * disabling the driver. This allows the controller to perform a smooth stop
   * before a driver disable enables the breaks.
   */
  static constexpr int DURATION_BETWEEN_HOLD_AND_DISABLE_MS{200};
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
AdapterStoTemplated<T>::AdapterStoTemplated(std::function<T(std::string)> create_service_client)
    : state_machine_(task_queue_),
      hold_srv_client_(create_service_client(HOLD_SERVICE)),
      unhold_srv_client_(create_service_client(UNHOLD_SERVICE)),
      recover_srv_client_(create_service_client(RECOVER_SERVICE)),
      halt_srv_client_(create_service_client(HALT_SERVICE)),
      is_executing_srv_client_(create_service_client(IS_EXECUTING_SERVICE))
{
  worker_thread_ = std::thread(&AdapterStoTemplated<T>::workerThreadFun, this);

  std::cout << "start state machine" << std::endl;
  state_machine_.start();
}

template <class T>
AdapterStoTemplated<T>::~AdapterStoTemplated()
{
  std::cout << "stop state machine" << std::endl;
  state_machine_.stop();

  if (worker_thread_.joinable())
  {
    terminate_ = true;
    worker_thread_.join();
  }
}

template <class T>
void AdapterStoTemplated<T>::updateSto(bool sto)
{
  {
    std::lock_guard<std::mutex> lock(sm_mutex_);
    state_machine_.process_event(typename StateMachine::sto_updated(sto));
  }
  worker_cv_.notify_one();
}

template <class T>
void AdapterStoTemplated<T>::workerThreadFun()
{
  std::unique_lock<std::mutex> sm_lock(sm_mutex_);
  while (!terminate_)
  {
    // wait for notification if there is no task to perform
    worker_cv_.wait(sm_lock, [this]() { return (!this->task_queue.empty()); });
    Task task = task_queue_.pop();

    // release lock during execution
    sm_lock.unlock();
    CompletionEvent event = task();
    sm_lock.lock();

    state_machine_.process_event(event);
  }
}

template <class T>
void AdapterStoTemplated<T>::call_recover()
{
  std_srvs::Trigger recover_trigger;
  ROS_ERROR_STREAM("Calling Recover (Service: " << recover_srv_client_.getService() << ")");
  bool recover_success = recover_srv_client_.call(recover_trigger);
  ROS_ERROR_STREAM("Finished Recover (Service: " << recover_srv_client_.getService() << ")");

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
  ROS_ERROR_STREAM("Calling Hold on controller (Service: " << hold_srv_client_.getService() << ")");
  bool hold_success = hold_srv_client_.call(hold_trigger);

  if (!hold_success)
  {
    ROS_ERROR_STREAM("No success calling Hold on controller (Service: " << hold_srv_client_.getService() << ")");
  }

  // check if hold trajectory is executed
  std_srvs::Trigger is_executing_trigger;
  bool is_executing_success = is_executing_srv_client_.call(is_executing_trigger);
  if (!is_executing_success)
  {
    ROS_ERROR_STREAM("No success calling service " << is_executing_srv_client_.getService());
  }
  if (is_executing_trigger.response.success || !is_executing_success)
  {
    ros::Duration(DURATION_BETWEEN_HOLD_AND_DISABLE_MS / 1000.0).sleep(); // wait until hold traj is finished
  }
}

template <class T>
void AdapterStoTemplated<T>::call_halt()
{
  std_srvs::Trigger halt_trigger;
  ROS_ERROR_STREAM("Calling Halt on driver (Service: " << halt_srv_client_.getService() << ")");
  bool halt_success = halt_srv_client_.call(halt_trigger);

  if (!halt_success)
  {
    ROS_ERROR_STREAM("No success calling Halt on driver (Service: " << halt_srv_client_.getService() << ")");
  }
}

} // namespace prbt_hardware_support

#endif // PRBT_HARDWARE_SUPPORT_ADAPTER_STO_H
