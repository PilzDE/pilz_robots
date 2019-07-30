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

#include <mutex>
#include <thread>
#include <string>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include <ros/time.h>
#include <ros/service_client.h>
#include <std_srvs/Trigger.h>

#include <prbt_hardware_support/service_client_factory.h>

namespace prbt_hardware_support
{

namespace msm = boost::msm;
namespace mpl = boost::mpl;
using namespace msm::front;

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
struct AdapterStoTemplated_ : public msm::front::state_machine_def<AdapterStoTemplated_<T>>
{
public:
  /**
   * @brief Connect to services.
   */
  AdapterStoTemplated_(std::function<T(std::string)> create_service_client = ServiceClientFactory::create<std_srvs::Trigger>);

  /**
   * @brief Trigger termination and join possible running threads.
   */
  ~AdapterStoTemplated_();

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

  bool sto_{false};
  bool stop_requested_{false};
  std::mutex event_mutex_;

//######################################################################################################################

  ////////////
  // States //
  ////////////

  struct RobotInactive : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
      std::cout << "entering: RobotInactive" << std::endl;
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
      std::cout << "leaving: RobotInactive" << std::endl;
    }
  };
  struct RobotActive : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
      std::cout << "entering: RobotActive" << std::endl;
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
      std::cout << "leaving: RobotActive" << std::endl;
    }
  };

  struct Recovering : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
      std::cout << "entering: Recovering" << std::endl;
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
      std::cout << "leaving: Recovering" << std::endl;
    }
  };

  struct Halting : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
      std::cout << "entering: Halting" << std::endl;
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
      std::cout << "leaving: Halting" << std::endl;
    }
  };

  struct Holding : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
      std::cout << "entering: Holding" << std::endl;
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
      std::cout << "leaving: Holding" << std::endl;
    }
  };

  struct Unholding : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
      std::cout << "entering: Unholding" << std::endl;
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
      std::cout << "leaving: Unholding" << std::endl;
    }
  };

  // initial state
  typedef RobotInactive initial_state;

  ////////////
  // Events //
  ////////////

  struct sto_changed
  {
    sto_changed(bool sto)
      : sto_(sto)
    {};

    bool sto_;
  };
  struct recover_done {};
  struct unhold_done {};
  struct hold_done {};
  struct halt_done {};

  ////////////
  // Guards //
  ////////////

  struct sto_true
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const&, FSM& fsm, SourceState&, TargetState&)
    {
      return fsm.sto_;
    }
  };

  struct sto_false
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const&, FSM& fsm, SourceState&, TargetState&)
    {
      return !fsm.sto_;
    }
  };

  struct stop_requested
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const&, FSM& fsm, SourceState&, TargetState&)
    {
      return fsm.stop_requested_;
    }
  };

  struct no_stop_requested
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const&, FSM& fsm, SourceState&, TargetState&)
    {
      return !fsm.stop_requested_;
    }
  };

  /////////////
  // Actions //
  /////////////

  struct update_sto
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const& evt, FSM& fsm, SourceState&, TargetState&)
    {
      std::cout << "update_sto" << std::endl;
      fsm.sto_ = evt.sto_;
      fsm.stop_requested_ = fsm.stop_requested_ || !evt.sto_;
    }
  };

  struct start_recover
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const&, FSM& fsm, SourceState&, TargetState&)
    {
      std::cout << "start_recover" << std::endl;
      fsm.stop_requested_ = false;

      std::thread recover_thread(
        [&fsm]()
        {
          fsm.call_recover();
          std::lock_guard<std::mutex> lock(fsm.event_mutex_);
          fsm.process_event(recover_done());
        });

      recover_thread.detach();
    }
  };

  struct start_halt
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const&, FSM& fsm, SourceState&, TargetState&)
    {
      std::cout << "start_halt" << std::endl;

      std::thread halt_thread(
        [&fsm]()
        {
          fsm.call_halt();
          std::lock_guard<std::mutex> lock(fsm.event_mutex_);
          fsm.process_event(halt_done());
        });

      halt_thread.detach();
    }
  };

  struct start_hold
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const&, FSM& fsm, SourceState&, TargetState&)
    {
      std::cout << "start_hold" << std::endl;

      std::thread hold_thread(
        [&fsm]()
        {
          fsm.call_hold();
          std::lock_guard<std::mutex> lock(fsm.event_mutex_);
          fsm.process_event(hold_done());
        });

      hold_thread.detach();
    }
  };

  struct start_unhold
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const&, FSM& fsm, SourceState&, TargetState&)
    {
      std::cout << "start_unhold" << std::endl;

      std::thread unhold_thread(
        [&fsm]()
        {
          fsm.call_unhold();
          std::lock_guard<std::mutex> lock(fsm.event_mutex_);
          fsm.process_event(unhold_done());
        });

      unhold_thread.detach();
    }
  };

  /////////////////
  // Transitions //
  /////////////////

  struct transition_table : mpl::vector<
  //  Start          Event            Target         Action                           Guard
  // +--------------+----------------+--------------+--------------------------------+------------------+
  Row< RobotInactive, sto_changed    , Recovering   , ActionSequence_<mpl::vector<
                                                        update_sto, start_recover> > , none             >,
  Row< Recovering   , sto_changed    , none         , update_sto                     , none             >,
  Row< Recovering   , recover_done   , Unholding    , start_unhold                   , no_stop_requested>,
  Row< Recovering   , recover_done   , Halting      , start_halt                     , stop_requested   >,
  Row< Unholding    , sto_changed    , none         , update_sto                     , none             >,
  Row< Unholding    , unhold_done    , RobotActive  , none                           , no_stop_requested>,
  Row< Unholding    , unhold_done    , Holding      , start_hold                     , stop_requested   >,
  Row< RobotActive  , sto_changed    , Holding      , ActionSequence_<mpl::vector<
                                                        update_sto, start_hold> >    , none             >,
  Row< Holding      , sto_changed    , none         , update_sto                     , none             >,
  Row< Holding      , hold_done      , Halting      , start_halt                     , none             >,
  Row< Halting      , sto_changed    , none         , update_sto                     , none             >,
  Row< Halting      , halt_done      , Recovering   , start_recover                  , sto_true         >,
  Row< Halting      , halt_done      , RobotInactive, none                           , sto_false        >
  // +--------------+----------------+--------------+--------------------------------+------------------+
  > {};

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

private:
  /**
   * @brief Specifies the time between holding the controller and
   * disabling the driver. This allows the controller to perform a smooth stop
   * before a driver disable enables the breaks.
   */
  static constexpr int DURATION_BETWEEN_HOLD_AND_DISABLE_MS{200};
};

// AdapterSto = state machine back end
template <class T = ros::ServiceClient>
using AdapterStoTemplated = msm::back::state_machine<AdapterStoTemplated_<T>>;

using AdapterSto = AdapterStoTemplated<>;

template <class T>
const std::string AdapterStoTemplated_<T>::HOLD_SERVICE{"manipulator_joint_trajectory_controller/hold"};

template <class T>
const std::string AdapterStoTemplated_<T>::UNHOLD_SERVICE{"manipulator_joint_trajectory_controller/unhold"};

template <class T>
const std::string AdapterStoTemplated_<T>::RECOVER_SERVICE{"driver/recover"};

template <class T>
const std::string AdapterStoTemplated_<T>::HALT_SERVICE{"driver/halt"};

template <class T>
const std::string AdapterStoTemplated_<T>::IS_EXECUTING_SERVICE{"manipulator_joint_trajectory_controller/is_executing"};

template <class T>
AdapterStoTemplated_<T>::AdapterStoTemplated_(std::function<T(std::string)> create_service_client)
    : hold_srv_client_(create_service_client(HOLD_SERVICE)),
      unhold_srv_client_(create_service_client(UNHOLD_SERVICE)),
      recover_srv_client_(create_service_client(RECOVER_SERVICE)),
      halt_srv_client_(create_service_client(HALT_SERVICE)),
      is_executing_srv_client_(create_service_client(IS_EXECUTING_SERVICE))
{
}

template <class T>
AdapterStoTemplated_<T>::~AdapterStoTemplated_()
{
}

template <class T>
void AdapterStoTemplated_<T>::call_recover()
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
void AdapterStoTemplated_<T>::call_unhold()
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
void AdapterStoTemplated_<T>::call_hold()
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
void AdapterStoTemplated_<T>::call_halt()
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
