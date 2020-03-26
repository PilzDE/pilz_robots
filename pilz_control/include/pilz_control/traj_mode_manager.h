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

#ifndef TRAJPROCESSINGMODEMANAGER_H
#define TRAJPROCESSINGMODEMANAGER_H

#include <list>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <unordered_map>

namespace pilz_joint_trajectory_controller
{

enum class TrajProcessingMode
{
  //! The controller accepts new trajectories.
  unhold,
  //! The controller rejects all new incoming trajectories and stops the robot.
  stopping,
  //! The robot stands still and the controller rejects all new incoming trajectories.
  hold
};

/**
 * @brief Stores the TrajProcessingMode state machine and can be used to determine if a transition is valid.
 */
class TrajProcessingModeStateMachine
{
public:
  /**
   * @brief Returns true if a transition between the current_mode and the requested_mode exists,
   * otherwise false.
   */
  bool isTransitionValid(const TrajProcessingMode& current_mode,
                         const TrajProcessingMode& requested_mode) const;

private:
  // "Simple" linear state machine: (start with stopping in accordance to the controller)
  //  x ->  |->  stopping  ->  hold  ->  unhold ->|
  //        |<-               <-                <-|
  const std::unordered_map<TrajProcessingMode, TrajProcessingMode> mode_machine_
  {
    {TrajProcessingMode::stopping,  TrajProcessingMode::hold},
    {TrajProcessingMode::hold,      TrajProcessingMode::unhold},
    {TrajProcessingMode::unhold,    TrajProcessingMode::stopping},
  };
};

/**
 * @brief Listener to wait for a specified mode to be reached.
 */
class TrajProcessingModeListener
{
public:
  //! @param mode Mode to wait for.
  TrajProcessingModeListener(const TrajProcessingMode& mode);
  void waitForMode();
  //! @returns true if the given mode corresponds to the target mode, otherwise false.
  bool isTargetModeReached(const TrajProcessingMode& mode) const;
  //! @brief Notify the listener that the target mode is reached.
  void triggerListener();

private:
  std::mutex mutex_;
  std::condition_variable cond_variable_;
  bool cond_fulfilled_ {false};
  //! @brief Mode to wait for.
  const TrajProcessingMode mode_;
};

/**
 * @brief Encapsulates a state machine managing the current Trajectory-Processing-Mode.
 */
class TrajProcessingModeManager
{
public:
  //! @returns true only if a successful state switch to stopping was performed, otherwise false.
  bool stoppingEvent();
  //! @brief Switch to hold.
  void stopMotionFinishedEvent();
  //! @returns true if in state unhold or a successful switch to state unhold was performed, otherwise false.
  bool unholdEvent();

public:
  //! @brief Check if in state stopping or hold.
  bool isHolding();
  bool isUnhold();
  void registerListener(TrajProcessingModeListener* const listener);

  TrajProcessingMode getCurrentMode();

private:
  //! @brief Perform full transition if possible.
  bool switchTo(const TrajProcessingMode& mode, const bool success_at_transition_only=true);
  bool setMode(const TrajProcessingMode& requested_mode, const bool& success_at_transition_only);
  //! @brief Triggers all registered listeners whose target mode is reached.
  void callListener(const TrajProcessingMode& mode);
  //! @brief Use only if lock on \ref mode_mutex_ is already acquired.

private:
  const TrajProcessingModeStateMachine mode_state_machine_;
  TrajProcessingMode current_mode_ {TrajProcessingMode::stopping};
  //! @brief Protects the access to the current mode.
  std::mutex mode_mutex_;

  std::list<TrajProcessingModeListener*> listener_;
  //! @brief Protects the access to the listener list.
  std::mutex listener_mutex_;
};

inline bool TrajProcessingModeStateMachine::isTransitionValid(const TrajProcessingMode& current_mode,
                                                              const TrajProcessingMode& requested_mode) const
{
  return mode_machine_.at(current_mode) == requested_mode;
}

inline TrajProcessingModeListener::TrajProcessingModeListener(const TrajProcessingMode& mode)
  : mode_(mode)
{
}

inline void TrajProcessingModeListener::waitForMode()
{
  std::unique_lock<std::mutex> lk(mutex_);
  while(!cond_fulfilled_)
  {
    cond_variable_.wait(lk);
  }
}

inline bool TrajProcessingModeListener::isTargetModeReached(const TrajProcessingMode& mode) const
{
  return mode == mode_;
}

inline void TrajProcessingModeListener::triggerListener()
{
  std::lock_guard<std::mutex> lk(mutex_);
  cond_fulfilled_ = true;
  cond_variable_.notify_one();
}

inline bool TrajProcessingModeManager::isHolding()
{
  TrajProcessingMode current_mode {getCurrentMode()};
  return (current_mode == TrajProcessingMode::stopping) || (current_mode == TrajProcessingMode::hold);
}

inline bool TrajProcessingModeManager::isUnhold()
{
  return getCurrentMode() == TrajProcessingMode::unhold;
}

inline TrajProcessingMode TrajProcessingModeManager::getCurrentMode()
{
  std::lock_guard<std::mutex> lk(mode_mutex_);
  return current_mode_;
}

inline bool TrajProcessingModeManager::setMode(const TrajProcessingMode& requested_mode,
                                               const bool& success_at_transition_only)
{
  std::lock_guard<std::mutex> lk(mode_mutex_);
  if (!success_at_transition_only && (current_mode_ == requested_mode))
  {
    return true;
  }

  if ( !mode_state_machine_.isTransitionValid(current_mode_, requested_mode) )
  {
    return false;
  }
  current_mode_ = requested_mode;
  return true;
}

inline void TrajProcessingModeManager::callListener(const TrajProcessingMode& mode)
{
  std::lock_guard<std::mutex> lk(listener_mutex_);
  std::list<TrajProcessingModeListener*>::iterator it = listener_.begin();
  while(it != listener_.end())
  {
    TrajProcessingModeListener* listener {(*it)};
    if (listener && listener->isTargetModeReached(mode))
    {
      listener->triggerListener();
      it = listener_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

inline bool TrajProcessingModeManager::switchTo(const TrajProcessingMode& mode,
                                                const bool success_at_transition_only)
{
  if (setMode(mode, success_at_transition_only))
  {
    callListener(mode);
    return true;
  }
  return false;
}

inline bool TrajProcessingModeManager::stoppingEvent()
{
  return switchTo(TrajProcessingMode::stopping);
}

inline void TrajProcessingModeManager::stopMotionFinishedEvent()
{
  switchTo(TrajProcessingMode::hold);
}

inline bool TrajProcessingModeManager::unholdEvent()
{
  return switchTo(TrajProcessingMode::unhold, false);
}

inline void TrajProcessingModeManager::registerListener(TrajProcessingModeListener* const listener)
{
  if (listener && listener->isTargetModeReached(getCurrentMode()))
  {
    listener->triggerListener();
    return;
  }
  std::lock_guard<std::mutex> lk(listener_mutex_);
  listener_.emplace_back(listener);
}


}

#endif // TRAJPROCESSINGMODEMANAGER_H
