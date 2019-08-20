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
  AdapterStoTemplated(const std::function<T(std::string, bool)> &create_service_client = ServiceClientFactory::create<std_srvs::Trigger>);

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
  void callRecoverService();

  /**
   * @brief Call the service triggering halt of the drives.
   */
  void callHaltService();

  /**
   * @brief Call the service triggering hold of the controller.
   */
  void callHoldService();

  /**
   * @brief Call the service triggering unhold of the controller.
   */
  void callUnholdService();

public:
  static const std::string HOLD_SERVICE;
  static const std::string UNHOLD_SERVICE;
  static const std::string RECOVER_SERVICE;
  static const std::string HALT_SERVICE;

protected:

  /**
   * @brief Stop the state machine.
   *
   * @note The access modifier protected allows this method to be used in tests.
   */
  void stopStateMachine();

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

  //! Mutex for protecting access to the state machine, needs to be owned when triggering an event of the state machine
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
AdapterStoTemplated<T>::AdapterStoTemplated(const std::function<T(std::string, bool)> &create_service_client)
    : hold_srv_client_(create_service_client(HOLD_SERVICE, false)),
      unhold_srv_client_(create_service_client(UNHOLD_SERVICE, false)),
      recover_srv_client_(create_service_client(RECOVER_SERVICE, false)),
      halt_srv_client_(create_service_client(HALT_SERVICE, false))
{
  state_machine_ = std::unique_ptr<StoStateMachine>(new StoStateMachine(
    std::bind(&AdapterStoTemplated<T>::callRecoverService, this),
    std::bind(&AdapterStoTemplated<T>::callHaltService, this),
    std::bind(&AdapterStoTemplated<T>::callHoldService, this),
    std::bind(&AdapterStoTemplated<T>::callUnholdService, this)));

  state_machine_->start();

  worker_thread_ = std::thread(&AdapterStoTemplated<T>::workerThreadFun, this);
}

template <class T>
AdapterStoTemplated<T>::~AdapterStoTemplated()
{
  if (worker_thread_.joinable())
  {
    {
      std::lock_guard<std::mutex> lock(sm_mutex_);
      terminate_ = true;
    }
    worker_cv_.notify_one();
    worker_thread_.join();
  }

  stopStateMachine();
}

template <class T>
void AdapterStoTemplated<T>::updateSto(const bool sto)
{
  ROS_DEBUG_STREAM("updateSto(" << std::boolalpha << sto << std::noboolalpha << ")");
  {
    std::lock_guard<std::mutex> lock(sm_mutex_);
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
    worker_cv_.wait(sm_lock, [this]() { return (!this->state_machine_->task_queue_.empty() || this->terminate_); });
    if (terminate_)
    {
      break;
    }

    AsyncStoTask task = state_machine_->task_queue_.front();
    state_machine_->task_queue_.pop();

    sm_lock.unlock();               // | This part is executed async from
    task.execute();                 // | the state machine since new sto updates need to be handled
                                    // | during service calls.
    sm_lock.lock();                 // |

    task.signalCompletion();  //Could add Task to Queue and does process_event on the state machine. Needs lock.
  }
}

template <class T>
void AdapterStoTemplated<T>::callRecoverService()
{
  std_srvs::Trigger recover_trigger;
  ROS_DEBUG_STREAM("Calling Recover (Service: " << recover_srv_client_.getService() << ")");
  bool recover_success = recover_srv_client_.call(recover_trigger);

  if (!recover_success)
  {
    ROS_ERROR_STREAM("No success calling Recover (Service: " << recover_srv_client_.getService() << ")");
  }
}

template <class T>
void AdapterStoTemplated<T>::callUnholdService()
{
  std_srvs::Trigger unhold_trigger;
  ROS_DEBUG_STREAM("Calling Unhold (Service: " << unhold_srv_client_.getService() << ")");
  bool unhold_success = unhold_srv_client_.call(unhold_trigger);

  if (!unhold_success)
  {
    ROS_ERROR_STREAM("No success calling Unhold (Service: " << unhold_srv_client_.getService() << ")" << "\n"
                     << "Most certainly a Stop1 is no longer feasible. "
                     << "Please beware that the hardware might get damaged.");
  }
}

template <class T>
void AdapterStoTemplated<T>::callHoldService()
{
  std_srvs::Trigger hold_trigger;
  ROS_DEBUG_STREAM("Calling Hold on controller (Service: " << hold_srv_client_.getService() << ")");
  bool hold_success = hold_srv_client_.call(hold_trigger);

  if (!hold_success)
  {
    ROS_ERROR_STREAM("No success calling Hold on controller (Service: " << hold_srv_client_.getService() << ")" << "\n"
                     << "Most certainly a Stop1 is no longer feasible. "
                     << "Please beware that the hardware might get damaged.");
  }
}

template <class T>
void AdapterStoTemplated<T>::callHaltService()
{
  std_srvs::Trigger halt_trigger;
  ROS_DEBUG_STREAM("Calling Halt on driver (Service: " << halt_srv_client_.getService() << ")");
  bool halt_success = halt_srv_client_.call(halt_trigger);

  if (!halt_success)
  {
    ROS_ERROR_STREAM("No success calling Halt on driver (Service: " << halt_srv_client_.getService() << ")");
  }
}

template <class T>
void AdapterStoTemplated<T>::stopStateMachine()
{
  ROS_DEBUG("Stop state machine");
  state_machine_->stop();
}

} // namespace prbt_hardware_support

#endif // PRBT_HARDWARE_SUPPORT_ADAPTER_STO_H
