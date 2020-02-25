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

#include <array>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>

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

// "Simple" linear state machine:
//  x ->  |->  hold  ->  unhold  ->  stopping ->|
//        |<-               <-                <-|
static constexpr unsigned int NUM_MODES {3};
static constexpr std::array<TrajProcessingMode, NUM_MODES> mode_state_machine_{
  TrajProcessingMode::hold, TrajProcessingMode::unhold, TrajProcessingMode::stopping
};

inline static constexpr TrajProcessingMode getModeToIdx(const unsigned int& idx)
{
  return mode_state_machine_[idx];
}

inline static constexpr bool isTransitionValid(const TrajProcessingMode& requested_new_mode,
                                               const unsigned int& idx_of_request_new_mode)
{
  return requested_new_mode == getModeToIdx(idx_of_request_new_mode);
}

inline static constexpr unsigned int getNextIndex(const unsigned int& current_idx)
{
  return (current_idx+1) % NUM_MODES;
}

/**
 * @brief Listener to wait for a specified modes to be reached.
 */
class TrajProcessingModeListener
{
public:
  TrajProcessingModeListener(const TrajProcessingMode& mode);
  void waitForMode();
  bool isTargetModeReached(const TrajProcessingMode& mode) const;
  void triggerListener();

private:
  std::mutex mutex_;
  std::condition_variable cond_variable_;
  bool cond_fulfilled_ {false};
  const TrajProcessingMode mode_;
};

/**
 * @brief Encapsulates a state machine managing the current Trajectory-Processing-Mode.
 */
class TrajProcessingModeManager
{
public:
  //! @returns true only if the a successful state switch to stopping happend, otherwise false.
  bool stoppingEvent();
  void stopMotionFinishedEvent();
  //! @returns  true if a successful switch to state unhold was performed, otherwise false.
  bool unholdEvent();

public:
  bool isholding();
  bool isUnhold();
  void registerListener(TrajProcessingModeListener* const listener);

  TrajProcessingMode getCurrentMode();

private:
  bool switchTo(const TrajProcessingMode& mode, const bool success_at_transition_only=true);
  bool setMode(const TrajProcessingMode& mode, const bool& success_at_transition_only);
  void callListener(const TrajProcessingMode& mode);
  TrajProcessingMode getCurrentModeLockFree() const;

private:
  //! @brief "Pointer" to current mode.
  unsigned int current_mode_idx_ {0};
  //! @brief Protects the access to to the current mode.
  std::mutex mode_mutex_;

  std::atomic<TrajProcessingModeListener*> listener_ {nullptr};
};

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

inline bool TrajProcessingModeManager::isholding()
{
  TrajProcessingMode current_ {getCurrentMode()};
  return (current_ == TrajProcessingMode::stopping) || (current_ == TrajProcessingMode::hold);
}

inline bool TrajProcessingModeManager::isUnhold()
{
  return getCurrentMode() == TrajProcessingMode::unhold;
}

inline TrajProcessingMode TrajProcessingModeManager::getCurrentModeLockFree() const
{
  return getModeToIdx(current_mode_idx_);
}

inline TrajProcessingMode TrajProcessingModeManager::getCurrentMode()
{
  std::lock_guard<std::mutex> lk(mode_mutex_);
  return getCurrentModeLockFree();
}

inline bool TrajProcessingModeManager::setMode(const TrajProcessingMode& mode,
                                               const bool& success_at_transition_only)
{
  std::lock_guard<std::mutex> lk(mode_mutex_);
  if (!success_at_transition_only && (getCurrentModeLockFree() == mode))
  {
    return true;
  }

  const unsigned int new_idx {getNextIndex(current_mode_idx_)};
  if ( !isTransitionValid(mode, new_idx) )
  {
    return false;
  }
  current_mode_idx_ = new_idx;
  return true;
}

inline void TrajProcessingModeManager::callListener(const TrajProcessingMode& mode)
{
  TrajProcessingModeListener* listener {std::atomic_exchange<TrajProcessingModeListener*>(&listener_, nullptr)};
  if (listener && listener->isTargetModeReached(mode))
  {
    listener->triggerListener();
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
  listener_.store(listener);
}


}

#endif // TRAJPROCESSINGMODEMANAGER_H
