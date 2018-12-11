/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#ifndef ASYNC_TEST_H
#define ASYNC_TEST_H

#include <ros/ros.h>

#include <mutex>
#include <condition_variable>
#include <atomic>

#include <gtest/gtest.h>

namespace testing
{

#define ACTION_OPEN_BARRIER(str) ::testing::InvokeWithoutArgs([this](void){this->triggerClearEvent(str); return true;})
#define ACTION_OPEN_BARRIER_VOID(str) ::testing::InvokeWithoutArgs([this](void){this->triggerClearEvent(str);})

/**
 * @brief Test class that allows the handling of asynchronous test objects
 *
 * The class provides two basic function. AsyncTest::barricade and AsyncTest::triggerClearEvent
 * During the test setup gates between the steps with one or more clear events. Allow passing on by calling
 * triggerClearEvent after a test.
 */
class AsyncTest
{
  public:
    /**
     * @brief Triggeres a clear event. If a call to barricade is currently pending it will unblock as soon as all clear
     * events are triggered. Else the event is put on the waitlist. This waitlist is emptied upon a call to barricade.
     *
     * @param event The event that is triggered
     */
    void triggerClearEvent(std::string event);

    /**
     * @brief Will block until the event given by clear_event is triggered. Unblocks immediately, if the event was
     * triggered in advance.
     *
     * @param clear_event Event that allows the test to pass on
     */
    void barricade(std::string clear_event);

    /**
     * @brief Will block until all events given by clear_events are triggered. Events triggered in advance take effect,
     * too.
     *
     * @param clear_events List of events that allow the test to pass on
     */
    void barricade(std::initializer_list<std::string> clear_events);

  protected:
    std::mutex m_;
    std::condition_variable cv_;
    std::set<std::string> clear_events_ {};
    std::set<std::string> waitlist_ {};
};

void AsyncTest::triggerClearEvent(std::string event)
{
  std::lock_guard<std::mutex> lk(m_);
  if (clear_events_.empty())
  {
    waitlist_.insert(event);
  }
  else if (clear_events_.erase(event) < 1)
  {
    ROS_WARN_STREAM("Triggered event " << event << " despite not waiting for it.");
  }
  cv_.notify_one();
}

void AsyncTest::barricade(std::string clear_event)
{
  barricade({clear_event});
}

void AsyncTest::barricade(std::initializer_list<std::string> clear_events)
{
  std::unique_lock<std::mutex> lk(m_);
  std::copy_if(clear_events.begin(), clear_events.end(), std::inserter(clear_events_, clear_events_.end()),
               [this](std::string event){ return this->waitlist_.count(event) == 0; });
  waitlist_.clear();
  while(!clear_events_.empty())
  {
    cv_.wait(lk);
  }
}

// for better readability in tests
#define BARRIER(str) barricade(str)
#define BARRIER2(str1, str2) barricade(str1, str2)
}

#endif // ASYNC_TEST_H