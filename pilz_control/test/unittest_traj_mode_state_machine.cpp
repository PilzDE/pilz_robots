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
TEST(TrajModeStateMachineTest, testTransitionStoppingToHold)
{
  TrajProcessingModeStateMachine state_machine;
  EXPECT_TRUE(state_machine.isTransitionValid(TrajProcessingMode::stopping, TrajProcessingMode::hold));
}

TEST(TrajModeStateMachineTest, testTransitionHoldToUnhold)
{
  TrajProcessingModeStateMachine state_machine;
  EXPECT_TRUE(state_machine.isTransitionValid(TrajProcessingMode::hold, TrajProcessingMode::unhold));
}

TEST(TrajModeStateMachineTest, testTransitionUnholdToStopping)
{
  TrajProcessingModeStateMachine state_machine;
  EXPECT_TRUE(state_machine.isTransitionValid(TrajProcessingMode::unhold, TrajProcessingMode::stopping));
}

TEST(TrajModeStateMachineTest, testTransitionStoppingToUnhold)
{
  TrajProcessingModeStateMachine state_machine;
  EXPECT_FALSE(state_machine.isTransitionValid(TrajProcessingMode::stopping, TrajProcessingMode::stopping));
}

TEST(TrajModeStateMachineTest, testTransitionHoldToStopping)
{
  TrajProcessingModeStateMachine state_machine;
  EXPECT_FALSE(state_machine.isTransitionValid(TrajProcessingMode::hold, TrajProcessingMode::stopping));
}

TEST(TrajModeStateMachineTest, testTransitionUnholdToHold)
{
  TrajProcessingModeStateMachine state_machine;
  EXPECT_FALSE(state_machine.isTransitionValid(TrajProcessingMode::unhold, TrajProcessingMode::hold));
}

}  // namespace pilz_joint_trajectory_controller

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
