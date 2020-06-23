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

#include <chrono>
#include <functional>
#include <future>
#include <array>

#include <gtest/gtest.h>

#include <pilz_control/traj_mode_manager.h>

namespace pilz_joint_trajectory_controller
{
static constexpr int WAIT_FOR_RESULT_TIMEOUT{ 1 };

TEST(HoldModeListenerTest, testWaitAndTrigger)
{
  HoldModeListener listener;

  std::future<void> wait_future = std::async(std::launch::async, std::bind(&HoldModeListener::wait, &listener));

  std::chrono::seconds timeout{ std::chrono::seconds(WAIT_FOR_RESULT_TIMEOUT) };
  EXPECT_EQ(wait_future.wait_for(timeout), std::future_status::timeout);

  listener.triggerListener();
  EXPECT_EQ(wait_future.wait_for(timeout), std::future_status::ready);
}

TEST(HoldModeListenerTest, testTriggerBeforeWait)
{
  HoldModeListener listener;

  listener.triggerListener();

  std::future<void> wait_future = std::async(std::launch::async, std::bind(&HoldModeListener::wait, &listener));

  std::chrono::seconds timeout{ std::chrono::seconds(WAIT_FOR_RESULT_TIMEOUT) };
  EXPECT_EQ(wait_future.wait_for(timeout), std::future_status::ready);
}

}  // namespace pilz_joint_trajectory_controller

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
