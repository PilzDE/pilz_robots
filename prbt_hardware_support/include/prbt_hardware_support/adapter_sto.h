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
  /**
   * @brief Connect to services.
   */
  AdapterStoTemplated(std::function<T(std::string)> create_service_client = ServiceClientFactory::create<std_srvs::Trigger>);

  /**
   * @brief Trigger termination and join possible running threads.
   */
  ~AdapterStoTemplated();

public:
  static const std::string HOLD_SERVICE;
  static const std::string UNHOLD_SERVICE;
  static const std::string RECOVER_SERVICE;
  static const std::string HALT_SERVICE;
  static const std::string IS_EXECUTING_SERVICE;

protected:
  /**
   * @brief Stops the controller and halts the driver after.
   */
  void performStop();

  /**
   * @brief Recovers the drives and unholds controller. Returns after recover if STO has changed to false.
   *
   * If the recover service fails (and STO is still true), retries to recover.
   */
  void enable();

  /**
   * @brief Update stored STO value and react to STO changes.
   * Reactions:
   *  - STO == false:   call performStop()
   *  - STO == true:    call enable() in own thread such that it can be interrupted by a new update.
   */
  void updateSto(const bool sto);

private:
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

  //! Current STO value
  std::atomic_bool sto_{false};

  //! Thread object for executing enable()
  std::thread enable_thread_;

private:
  /**
   * @brief Specifies the time between holding the controller and
   * disabling the driver. This allows the controller to perform a smooth stop
   * before a driver disable enables the breaks.
   */
  static constexpr int DURATION_BETWEEN_HOLD_AND_DISABLE_MS{200};
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
const std::string AdapterStoTemplated<T>::IS_EXECUTING_SERVICE{"manipulator_joint_trajectory_controller/is_executing"};

template <class T>
AdapterStoTemplated<T>::AdapterStoTemplated(std::function<T(std::string)> create_service_client)
    : hold_srv_client_(create_service_client(HOLD_SERVICE)),
      unhold_srv_client_(create_service_client(UNHOLD_SERVICE)),
      recover_srv_client_(create_service_client(RECOVER_SERVICE)),
      halt_srv_client_(create_service_client(HALT_SERVICE)),
      is_executing_srv_client_(create_service_client(IS_EXECUTING_SERVICE))
{
}

template <class T>
AdapterStoTemplated<T>::~AdapterStoTemplated()
{
  sto_ = false;

  if (enable_thread_.joinable())
  {
    enable_thread_.join();
  }
}

template <class T>
void AdapterStoTemplated<T>::updateSto(const bool sto)
{
  ROS_ERROR_STREAM("UPDATING IN THREAD " << std::this_thread::get_id());
  if (sto == sto_)
  {
    ROS_ERROR_STREAM("NO UPDATE");
    return;
  }

  sto_ = sto;

  if (!sto)
  {
    performStop();
    return;
  }

  if (enable_thread_.joinable())
  {
    enable_thread_.join();
  }
  enable_thread_ = std::thread(&AdapterStoTemplated<T>::enable, this);
}

template <class T>
void AdapterStoTemplated<T>::enable()
{
  bool recover_success{false};
  ros::Rate rate(10);
  while (!recover_success)
  {
    std_srvs::Trigger recover_trigger;
    ROS_ERROR_STREAM("Calling Recover (Service: " << recover_srv_client_.getService() << ")");
    recover_success = recover_srv_client_.call(recover_trigger);
    ROS_ERROR_STREAM("Finished Recover (Service: " << recover_srv_client_.getService() << ")");

    if (!recover_success)
    {
      ROS_ERROR_STREAM("No success calling Recover (Service: " << recover_srv_client_.getService() << ")");
    }

    // Abort enabling. Do not leave hold mode of controller in case STO changes from TRUE -> FALSE during recover
    if (!sto_)
    {
      return;
    }

    rate.sleep();
  }

  std_srvs::Trigger unhold_trigger;
  ROS_DEBUG_STREAM("Calling Unhold (Service: " << unhold_srv_client_.getService() << ")");
  bool unhold_success = unhold_srv_client_.call(unhold_trigger);

  if (!unhold_success)
  {
    ROS_ERROR_STREAM("No success calling Unhold (Service: " << unhold_srv_client_.getService() << ")");
  }
}

template <class T>
void AdapterStoTemplated<T>::performStop()
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
