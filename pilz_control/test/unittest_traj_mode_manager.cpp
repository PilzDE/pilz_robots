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

static constexpr int WAIT_FOR_RESULT_TIMEOUT{1};

inline std::future<void> waitForModeAsync(TrajProcessingModeListener& listener)
{
  return std::async(std::launch::async, std::bind(&TrajProcessingModeListener::waitForMode, &listener));
}

inline bool modeReached(const std::future<void>& wait_future, int timeout = WAIT_FOR_RESULT_TIMEOUT)
{
  return wait_future.wait_for(std::chrono::seconds(timeout)) == std::future_status::ready;
}

TEST(TrajModeManagerTest, testInitialMode)
{
  TrajProcessingModeManager manager;
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);
}

TEST(TrajModeManagerTest, testUnholding)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  EXPECT_TRUE(manager.unholdEvent());
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::unhold);
  EXPECT_TRUE(manager.unholdEvent());
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::unhold);
}

TEST(TrajModeManagerTest, testStopping)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  EXPECT_FALSE(manager.stoppingEvent());

  ASSERT_TRUE(manager.unholdEvent());
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::unhold);

  EXPECT_TRUE(manager.stoppingEvent());
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);
  EXPECT_FALSE(manager.stoppingEvent());
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);
  EXPECT_FALSE(manager.unholdEvent());
}

TEST(TrajModeManagerTest, testStopFinished)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  manager.stopMotionFinishedEvent();
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  ASSERT_TRUE(manager.unholdEvent());
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::unhold);

  manager.stopMotionFinishedEvent();
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::unhold);

  ASSERT_TRUE(manager.stoppingEvent());
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);

  manager.stopMotionFinishedEvent();
  EXPECT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);
  EXPECT_FALSE(manager.stoppingEvent());
}

TEST(TrajModeManagerTest, testIsHolding)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  EXPECT_TRUE(manager.isHolding());

  ASSERT_TRUE(manager.unholdEvent());
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::unhold);

  EXPECT_FALSE(manager.isHolding());

  ASSERT_TRUE(manager.stoppingEvent());
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);

  EXPECT_TRUE(manager.isHolding());
}

TEST(TrajModeManagerTest, testIsUnhold)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  EXPECT_FALSE(manager.isUnhold());

  ASSERT_TRUE(manager.unholdEvent());
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::unhold);

  EXPECT_TRUE(manager.isUnhold());

  ASSERT_TRUE(manager.stoppingEvent());
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::stopping);

  EXPECT_FALSE(manager.isUnhold());
}

TEST(TrajModeManagerTest, testListenerModeAlreadyReached)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  TrajProcessingModeListener listener(TrajProcessingMode::hold);
  manager.registerListener(&listener);

  auto wait_future = waitForModeAsync(listener);
  EXPECT_TRUE(modeReached(wait_future));
}

TEST(TrajModeManagerTest, testListenerModeNotReached)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  TrajProcessingModeListener listener(TrajProcessingMode::unhold);
  manager.registerListener(&listener);

  auto wait_future = waitForModeAsync(listener);
  EXPECT_FALSE(modeReached(wait_future));

  listener.triggerListener();  // make sure wait finishes eventually
  EXPECT_TRUE(modeReached(wait_future));
}

TEST(TrajModeManagerTest, testListenerSwitchToMode)
{
  TrajProcessingModeManager manager;
  ASSERT_EQ(manager.getCurrentMode(), TrajProcessingMode::hold);

  TrajProcessingModeListener listener(TrajProcessingMode::unhold);
  manager.registerListener(&listener);

  auto wait_future = waitForModeAsync(listener);

  ASSERT_TRUE(manager.unholdEvent());
  EXPECT_TRUE(modeReached(wait_future));
}

}  // namespace pilz_joint_trajectory_controller

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
