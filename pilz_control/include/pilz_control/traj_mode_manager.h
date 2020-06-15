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
  bool isTransitionValid(const TrajProcessingMode& current_mode, const TrajProcessingMode& requested_mode) const;

private:
  // "Simple" linear state machine: (start with stopping in accordance to the controller)
  //  x ->  |->  stopping  ->  hold  ->  unhold ->|
  //        |<-               <-                <-|
  const std::unordered_map<TrajProcessingMode, TrajProcessingMode> mode_machine_{
    { TrajProcessingMode::stopping, TrajProcessingMode::hold },
    { TrajProcessingMode::hold, TrajProcessingMode::unhold },
    { TrajProcessingMode::unhold, TrajProcessingMode::stopping },
  };
};

/**
 * @brief Listener to wait for the hold mode to be reached.
 */
class HoldModeListener
{
public:
  void wait();
  //! @brief Notify the listener that the hold mode is reached.
  void triggerListener();

private:
  std::mutex mutex_;
  std::condition_variable cond_variable_;
  bool cond_fulfilled_{ false };
};

/**
 * @brief Encapsulates a state machine managing the current Trajectory-Processing-Mode.
 *
 * All public methods are mutually exclusive. Initial mode is stopping in accordance to the controller.
 */
class TrajProcessingModeManager
{
public:
  //! @return True only if a successful state switch to stopping was performed, otherwise false.
  bool stopEvent(HoldModeListener* const listener = nullptr);
  //! @brief Switch to hold.
  void stopMotionFinishedEvent();
  //! @return True if in state unhold or a successful switch to state unhold was performed, otherwise false.
  bool startEvent();

public:
  //! @brief Check if in state stopping or hold.
  bool isHolding();
  TrajProcessingMode getCurrentMode();

private:
  /**
   * @brief Perform transition if possible.
   * @return True if transition was performed, otherwise false.
   */
  bool switchTo(const TrajProcessingMode& mode);
  void registerListener(HoldModeListener* const listener);
  //! @brief Triggers all registered listeners whose target mode is reached.
  void callListener();

private:
  const TrajProcessingModeStateMachine mode_state_machine_{};
  TrajProcessingMode current_mode_{ TrajProcessingMode::stopping };
  std::list<HoldModeListener*> listener_;
  //! @brief Used to make all public methods mutually exclusive in order to protect the member variables.
  std::mutex mutex_;
};

inline bool TrajProcessingModeStateMachine::isTransitionValid(const TrajProcessingMode& current_mode,
                                                              const TrajProcessingMode& requested_mode) const
{
  return mode_machine_.at(current_mode) == requested_mode;
}

inline void HoldModeListener::wait()
{
  std::unique_lock<std::mutex> lk(mutex_);
  while (!cond_fulfilled_)
  {
    cond_variable_.wait(lk);
  }
}

inline void HoldModeListener::triggerListener()
{
  std::lock_guard<std::mutex> lk(mutex_);
  cond_fulfilled_ = true;
  cond_variable_.notify_one();
}

inline bool TrajProcessingModeManager::isHolding()
{
  TrajProcessingMode current_mode{ getCurrentMode() };
  return (current_mode == TrajProcessingMode::stopping) || (current_mode == TrajProcessingMode::hold);
}

inline TrajProcessingMode TrajProcessingModeManager::getCurrentMode()
{
  std::lock_guard<std::mutex> lk(mutex_);
  return current_mode_;
}

inline bool TrajProcessingModeManager::switchTo(const TrajProcessingMode& mode)
{
  if (mode_state_machine_.isTransitionValid(current_mode_, mode))
  {
    current_mode_ = mode;
    return true;
  }
  return false;
}

inline void TrajProcessingModeManager::registerListener(HoldModeListener* const listener)
{
  listener_.emplace_back(listener);
}

inline void TrajProcessingModeManager::callListener()
{
  std::list<HoldModeListener*>::iterator it = listener_.begin();
  while (it != listener_.end())
  {
    HoldModeListener* listener{ (*it) };
    if (listener)
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

inline bool TrajProcessingModeManager::stopEvent(HoldModeListener* const listener)
{
  std::lock_guard<std::mutex> lk(mutex_);
  bool transition_performed{ switchTo(TrajProcessingMode::stopping) };
  registerListener(listener);
  if (current_mode_ == TrajProcessingMode::hold)
  {
    callListener();
  }
  return transition_performed;
}

inline bool TrajProcessingModeManager::startEvent()
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (current_mode_ == TrajProcessingMode::unhold || switchTo(TrajProcessingMode::unhold))
  {
    return true;
  }
  return false;
}

inline void TrajProcessingModeManager::stopMotionFinishedEvent()
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (switchTo(TrajProcessingMode::hold))
  {
    callListener();
  }
}

}  // namespace pilz_joint_trajectory_controller

#endif  // TRAJPROCESSINGMODEMANAGER_H
