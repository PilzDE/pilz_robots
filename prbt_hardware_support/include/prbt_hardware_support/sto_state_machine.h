/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
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

#ifndef PRBT_HARDWARE_SUPPORT_STO_STATE_MACHINE_H
#define PRBT_HARDWARE_SUPPORT_STO_STATE_MACHINE_H

#include <string>

#include <ros/ros.h>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/variant.hpp>

#include <prbt_hardware_support/adapter_sto.h>

namespace prbt_hardware_support
{

namespace msm = boost::msm;
namespace mpl = boost::mpl;
using namespace msm::front;

/**
 * TODO
 */
template <class AdapterType>
class StoStateMachine_ : public msm::front::state_machine_def<StoStateMachine_<AdapterType>>
{
public:
  StoStateMachine_(std::shared_ptr<typename AdapterType::TaskQueue> task_queue)
    : task_queue_(task_queue) {};

  ////////////
  // States //
  ////////////

  struct RobotInactive : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
      ROS_ERROR("entering: RobotInactive");
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
      ROS_ERROR("leaving: RobotInactive");
    }
  };
  struct RobotActive : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
      ROS_ERROR("entering: RobotActive");
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
      ROS_ERROR("leaving: RobotActive");
    }
  };

  struct Enabling : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
      ROS_ERROR("entering: Enabling");
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
      ROS_ERROR("leaving: Enabling");
    }
  };

  struct Stopping : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
      ROS_ERROR("entering: Stopping");
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
      ROS_ERROR("leaving: Stopping");
    }
  };

  struct StopRequestedDuringEnable : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
      ROS_ERROR("entering: StopRequestedDuringEnable");
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
      ROS_ERROR("leaving: StopRequestedDuringEnable");
    }
  };

  struct EnableRequestedDuringStop : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&)
    {
      ROS_ERROR("entering: EnableRequestedDuringHalt");
    }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&)
    {
      ROS_ERROR("leaving: EnableRequestedDuringHalt");
    }
  };

  // initial state
  typedef RobotInactive initial_state;

  ////////////
  // Events //
  ////////////

  struct sto_updated
  {
    sto_updated(bool sto)
      : sto_(sto)
    {};

    bool sto_;
  };
  struct recover_done {};
  struct halt_done {};
  struct hold_done {};
  struct unhold_done {};

  ////////////
  // Guards //
  ////////////

  struct sto_true
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const& evt, FSM&, SourceState&, TargetState&)
    {
      return evt.sto_;
    }
  };

  struct sto_false
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const& evt, FSM&, SourceState&, TargetState&)
    {
      return !evt.sto_;
    }
  };

  /////////////
  // Actions //
  /////////////

  struct recover_start
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const&, FSM& fsm, SourceState&, TargetState&)
    {
      ROS_ERROR("recover_start");

      task_queue_.push(AdapterType::Task(&AdapterType::call_recover, recover_done()));
    }
  };

  struct halt_start
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const&, FSM& fsm, SourceState&, TargetState&)
    {
      ROS_ERROR("halt_start");

      task_queue_.push(AdapterType::Task(&AdapterType::call_halt, halt_done()));
    }
  };

  struct hold_start
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const&, FSM& fsm, SourceState&, TargetState&)
    {
      ROS_ERROR("hold_start");

      task_queue_.push(AdapterType::Task(&AdapterType::call_hold, hold_done()));
    }
  };

  struct unhold_start
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const&, FSM& fsm, SourceState&, TargetState&)
    {
      ROS_ERROR("unhold_start");

      task_queue_.push(AdapterType::Task(&AdapterType::call_unhold, unhold_done()));
    }
  };

  /////////////////
  // Transitions //
  /////////////////

  struct transition_table : mpl::vector<
  //  Start                       Event          Target                      Action         Guard
  // +---------------------------+--------------+---------------------------+--------------+----------+
  Row< RobotInactive             , sto_updated  , Enabling                  , recover_start, sto_true >,
  Row< RobotInactive             , sto_updated  , none                      , none         , sto_false>,
  Row< Enabling                  , sto_updated  , none                      , none         , sto_true >,
  Row< Enabling                  , sto_updated  , StopRequestedDuringEnable , none         , sto_false>,
  Row< Enabling                  , recover_done , none                      , unhold_start , none     >,
  Row< Enabling                  , unhold_done  , RobotActive               , none         , none     >,
  Row< StopRequestedDuringEnable , sto_updated  , none                      , none         , none     >,
  Row< StopRequestedDuringEnable , recover_done , Stopping                  , halt_start   , none     >,
  Row< StopRequestedDuringEnable , unhold_done  , Stopping                  , hold_start   , none     >,
  Row< RobotActive               , sto_updated  , none                      , none         , sto_true >,
  Row< RobotActive               , sto_updated  , Stopping                  , hold_start   , sto_false>,
  Row< Stopping                  , sto_updated  , EnableRequestedDuringStop , none         , sto_true >,
  Row< Stopping                  , sto_updated  , none                      , none         , sto_false>,
  Row< Stopping                  , hold_done    , none                      , halt_start   , none     >,
  Row< Stopping                  , halt_done    , RobotInactive             , none         , none     >,
  Row< EnableRequestedDuringStop , sto_updated  , none                      , none         , sto_true >,
  Row< EnableRequestedDuringStop , sto_updated  , Stopping                  , none         , sto_false>,
  Row< EnableRequestedDuringStop , hold_done    , Enabling                  , recover_start, none     >,
  Row< EnableRequestedDuringStop , halt_done    , Enabling                  , recover_start, none     >
  // +---------------------------+--------------+---------------------------+--------------+----------+
  > {};

  //! Task queue consists of single task
  std::shared_ptr<typename AdapterType::TaskQueue> task_queue_;
};

// top-level state machine
template <class AdapterType>
using StoStateMachine = msm::back::state_machine<StoStateMachine_<AdapterType>>;

} // namespace prbt_hardware_support

#endif // PRBT_HARDWARE_SUPPORT_STO_STATE_MACHINE_H
