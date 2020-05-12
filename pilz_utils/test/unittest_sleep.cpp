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

#include <ros/ros.h>

#include <pilz_utils/sleep.h>

namespace pilz_utils
{
static constexpr double DEFAULT_SLEEP_TIME_S{ 0.1 };
static constexpr double DEFAULT_STARTING_TIME_S{ 0.1 };
static constexpr double TIME_COMPARISON_TOLERANCE_S{ 0.000001 };

static constexpr double INVALID_SIMULATION_TIME_S{ 0.0 };

TEST(SleepTest, testSleepSimTime)
{
  ros::Time::init();
  ros::Time starting_time{ DEFAULT_STARTING_TIME_S };
  ros::Time::setNow(starting_time);

  ros::Duration sleep_time{ DEFAULT_SLEEP_TIME_S };
  EXPECT_TRUE(pilz_utils::sleep(sleep_time)) << "The function sleep did not return true as expected.";

  EXPECT_NEAR(ros::Time::now().toSec(), (starting_time + sleep_time).toSec(), TIME_COMPARISON_TOLERANCE_S) << "Simulati"
                                                                                                              "on time "
                                                                                                              "did not "
                                                                                                              "progress"
                                                                                                              " as "
                                                                                                              "expected"
                                                                                                              " during "
                                                                                                              "sleep.";
}

TEST(SleepTest, testSleepInvalidSimTime)
{
  ros::Time::init();
  ros::Time starting_time{ INVALID_SIMULATION_TIME_S };
  ros::Time::setNow(starting_time);

  ros::Duration sleep_time{ DEFAULT_SLEEP_TIME_S };
  EXPECT_FALSE(pilz_utils::sleep(sleep_time)) << "The function sleep did not return false as expected.";

  EXPECT_NEAR(ros::Time::now().toSec(), starting_time.toSec(), TIME_COMPARISON_TOLERANCE_S) << "Simulation time did "
                                                                                               "progress unexpectedly "
                                                                                               "during sleep.";
}

/**
 * For stability reasons this test only checks if sleep() lasts long enough.
 */
TEST(SleepTest, testSleepSystemTime)
{
  ros::Time::init();
  ros::Duration sleep_time{ DEFAULT_SLEEP_TIME_S };

  ros::Time starting_time{ ros::Time::now() };
  EXPECT_TRUE(pilz_utils::sleep(sleep_time)) << "The function sleep did not return true as expected.";
  ros::Time end_time{ ros::Time::now() };

  EXPECT_GT((end_time - starting_time).toSec(), sleep_time.toSec()) << "Sleep ended too early.";
}

}  // namespace pilz_utils

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
