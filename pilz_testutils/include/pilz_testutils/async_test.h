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

#include <mutex>
#include <condition_variable>

#include <gtest/gtest.h>

namespace testing
{

/**
 * @brief Test class that allows the handling of asynchronous test objects
 *
 * The class provides the two basic functions AsyncTest::barricade and AsyncTest::triggerClearEvent.
 * During the test setup gates between the steps with one or more clear events. Allow passing on by calling
 * triggerClearEvent after a test.
 *
 * \e Usage:<br>
 * Suppose you want to test a function that calls another function asynchronously, like the following example:
 *
 * \code
 * void asyncCall(std::function<void()> fun)
 * {
 *     std::thread t(fun);
 *     t.detach();
 * }
 * \endcode
 *
 * You expect that fun gets called, so your test thread has to wait for the completion, else it would fail. This can be
 * achieved via:
 *
 * \code
 * class MyTest : public testing::Test, public testing::AsyncTest
 * {
 * public:
 *     MOCK_METHOD0(myMethod, void());
 * };
 *
 * TEST_F(MyTest, testCase)
 * {
 *     EXPECT_CALL(*this, myMethod()).Times(1).WillOnce(ACTION_OPEN_BARRIER_VOID("myMethod"));
 *     asyncCall(std::bind(&MyTest::myMethod, this));
 *     BARRIER("myMethod");
 * }
 * \endcode
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

// for better readability in tests
#define BARRIER(...) barricade(__VA_ARGS__)
#define BARRIER2(str1, str2) barricade(str1, str2)

#define ACTION_OPEN_BARRIER(str) ::testing::InvokeWithoutArgs([this](void){this->triggerClearEvent(str); return true;})
#define ACTION_OPEN_BARRIER_VOID(str) ::testing::InvokeWithoutArgs([this](void){this->triggerClearEvent(str);})
}

#endif // ASYNC_TEST_H
