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

#include <gtest/gtest.h>

#include <pilz_control/traj_mode_manager.h>

namespace pilz_joint_trajectory_controller
{

TEST(TrajModeStateMachineTest, testIsTransitionValid)
{
  for (unsigned int i = 0; i < NUM_MODES; ++i)
  {
    auto mode = getMode(i);
    EXPECT_TRUE(isTransitionValid(mode, i));
  }
}

TEST(TrajModeStateMachineTest, testNextTransitionInHoldMode)
{
  TrajProcessingMode mode{TrajProcessingMode::hold};
  bool found_index{false};
  for (unsigned int i = 0; i < NUM_MODES; ++i)
  {
    if (isTransitionValid(mode, i))
    {
      found_index = true;
      auto next_index = getNextIndex(i);
      EXPECT_TRUE(isTransitionValid(TrajProcessingMode::unhold, next_index));
    }
  }
  EXPECT_TRUE(found_index);
}

TEST(TrajModeStateMachineTest, testNextTransitionInUnholdMode)
{
  TrajProcessingMode mode{TrajProcessingMode::unhold};
  bool found_index{false};
  for (unsigned int i = 0; i < NUM_MODES; ++i)
  {
    if (isTransitionValid(mode, i))
    {
      found_index = true;
      auto next_index = getNextIndex(i);
      EXPECT_TRUE(isTransitionValid(TrajProcessingMode::stopping, next_index));
    }
  }
  EXPECT_TRUE(found_index);
}

TEST(TrajModeStateMachineTest, testNextTransitionInStoppingMode)
{
  TrajProcessingMode mode{TrajProcessingMode::stopping};
  bool found_index{false};
  for (unsigned int i = 0; i < NUM_MODES; ++i)
  {
    if (isTransitionValid(mode, i))
    {
      found_index = true;
      auto next_index = getNextIndex(i);
      EXPECT_TRUE(isTransitionValid(TrajProcessingMode::hold, next_index));
    }
  }
  EXPECT_TRUE(found_index);
}

}  // namespace pilz_joint_trajectory_controller

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
