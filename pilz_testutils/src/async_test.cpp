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

#include <algorithm>
#include <chrono>

#include <ros/ros.h>

#include <pilz_testutils/async_test.h>

namespace testing
{

bool AsyncTest::barricade(const std::string& clear_event, const int timeout_ms)
{
  return barricade({clear_event}, timeout_ms);
}

bool AsyncTest::barricade(std::initializer_list<std::string> clear_events, const int timeout_ms)
{
  std::unique_lock<std::mutex> lk(m_);

  std::stringstream events_stringstream;
  for (const auto& event : clear_events)
  {
    events_stringstream << event << ", ";
  }
  ROS_DEBUG_NAMED("Test", "Adding Barricade[%s]", events_stringstream.str().c_str());

  std::copy_if(clear_events.begin(), clear_events.end(), std::inserter(clear_events_, clear_events_.end()),
               [this](std::string event){ return this->waitlist_.count(event) == 0; });
  waitlist_.clear();

  auto end_time_point = std::chrono::system_clock::now() + std::chrono::milliseconds(timeout_ms);

  while(!clear_events_.empty())
  {
    if (timeout_ms < 0)
    {
      cv_.wait(lk);
    }
    else
    {
      std::cv_status status = cv_.wait_for(lk, end_time_point - std::chrono::system_clock::now());
      if (status == std::cv_status::timeout)
      {
        return clear_events_.empty();
      }
    }
  }
  return true;
}

void AsyncTest::triggerClearEvent(const std::string& event)
{
  std::lock_guard<std::mutex> lk(m_);

  if (clear_events_.empty())
  {
    ROS_DEBUG_NAMED("Test", "Clearing Barricade[%s]", event.c_str());
    waitlist_.insert(event);
  }
  else if (clear_events_.erase(event) < 1)
  {
    ROS_WARN_STREAM("Triggered event " << event << " despite not waiting for it.");
  }
  cv_.notify_one();
}

} // namespace testing
