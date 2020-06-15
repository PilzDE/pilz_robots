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

#include <gtest/gtest.h>

#include <pilz_control/traj_mode_manager.h>

namespace pilz_joint_trajectory_controller
{
static constexpr int WAIT_FOR_RESULT_TIMEOUT{ 1 };

inline std::future<void> waitAsync(HoldModeListener& listener)
{
  return std::async(std::launch::async, std::bind(&HoldModeListener::wait, &listener));
}

inline bool holdReached(const std::future<void>& wait_future, int timeout = WAIT_FOR_RESULT_TIMEOUT)
{
  return wait_future.wait_for(std::chrono::seconds(timeout)) == std::future_status::ready;
}

TEST(TrajModeManagerTest, testStoppingMode)
{
  TrajProcessingModeManager manager;
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);

  EXPECT_FALSE(manager.startEvent());
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);

  EXPECT_FALSE(manager.stopEvent());
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);
}

TEST(TrajModeManagerTest, testHoldMode)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);

  manager.stopMotionFinishedEvent();
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  manager.stopMotionFinishedEvent();
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  EXPECT_FALSE(manager.stopEvent());
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);
}

TEST(TrajModeManagerTest, testUnholdMode)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);

  manager.stopMotionFinishedEvent();
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  EXPECT_TRUE(manager.startEvent());
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::unhold);

  EXPECT_TRUE(manager.startEvent());
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::unhold);

  manager.stopMotionFinishedEvent();
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::unhold);
}

TEST(TrajModeManagerTest, testCycleThroughModes)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);

  const unsigned int number_of_iterations{ 3U };
  for (unsigned int i = 0; i < number_of_iterations; ++i)
  {
    EXPECT_FALSE(manager.stopEvent());
    EXPECT_FALSE(manager.startEvent());
    manager.stopMotionFinishedEvent();
    EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

    EXPECT_FALSE(manager.stopEvent());
    EXPECT_TRUE(manager.startEvent());
    EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::unhold);

    EXPECT_TRUE(manager.startEvent());
    EXPECT_TRUE(manager.stopEvent());
    EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);
  }
}

TEST(TrajModeManagerTest, testIsHolding)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);

  EXPECT_TRUE(manager.isHolding());

  manager.stopMotionFinishedEvent();
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  EXPECT_TRUE(manager.isHolding());

  ASSERT_TRUE(manager.startEvent());
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::unhold);

  EXPECT_FALSE(manager.isHolding());
}

TEST(TrajModeManagerTest, testListenerModeAlreadyReached)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);

  manager.stopMotionFinishedEvent();
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  HoldModeListener listener;
  EXPECT_FALSE(manager.stopEvent(&listener));

  auto wait_future = waitAsync(listener);
  EXPECT_TRUE(holdReached(wait_future));
}

TEST(TrajModeManagerTest, testListenerModeNotReached)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);

  HoldModeListener listener;
  EXPECT_FALSE(manager.stopEvent(&listener));

  auto wait_future = waitAsync(listener);
  EXPECT_FALSE(holdReached(wait_future));

  listener.triggerListener();  // make sure wait finishes eventually
  EXPECT_TRUE(holdReached(wait_future));
}

TEST(TrajModeManagerTest, testListenerSwitchToMode)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);

  HoldModeListener listener;
  EXPECT_FALSE(manager.stopEvent(&listener));
  auto wait_future = waitAsync(listener);

  manager.stopMotionFinishedEvent();
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  EXPECT_TRUE(holdReached(wait_future));
}

}  // namespace pilz_joint_trajectory_controller

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
