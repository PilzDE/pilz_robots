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

#define ACTION_OPEN_BARRIER(num) ::testing::InvokeWithoutArgs([this](void){this->enableTestStep(num); return true;})
#define ACTION_OPEN_BARRIER_VOID(num) ::testing::InvokeWithoutArgs([this](void){this->enableTestStep(num);})

/**
 * @brief Test class that allows the handling of asynchronous test objects
 *
 * The class provides two basic function. AsyncTest::barricadeTestStep and AsyncTest::setTestStep
 * during the test setup gates between the steps and trigger a setTestStep to allow passing on after a test.
 */
class AsyncTest
{
  public:
    /**
     * @brief Allows the test step @param step to pass. If a call to barricadeTestStep with the requested
     * step is currently pending it will now unblock.
     *
     * @param step The test step to enable
     */
    void enableTestStep(int step);

    /**
     * @brief If no call to setTestStep with the requested step was set before it will block until a enableTestStep
     * with the required step is called
     *
     * @param step The test step that is gated
     */
    void barricadeTestStep(int step);

  protected:
    std::mutex m_;
    std::condition_variable cv_;
    std::atomic_int test_step_{-1};
};

void AsyncTest::enableTestStep(int step)
{
  std::lock_guard<std::mutex> lk(m_);
  test_step_ = step;
  cv_.notify_one();
}

void AsyncTest::barricadeTestStep(int step)
{
  while(test_step_ != step)
  {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait(lk);
  }
}

// for better readability in tests
#define BARRIER_STEP(i) barricadeTestStep(i)
}

#endif // ASYNC_TEST_H