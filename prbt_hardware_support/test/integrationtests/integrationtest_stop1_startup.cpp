/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include <functional>
#include <map>
#include <string>

#include <ros/ros.h>

#include <prbt_hardware_support/pilz_manipulator_mock.h>
#include <prbt_hardware_support/ros_test_helper.h>

namespace prbt_hardware_support
{

static const std::string RECOVER_SERVICE_NAME {"recover"};
static const std::string HALT_SERVICE_NAME {"halt"};
static const std::string HOLD_SERVICE_NAME {"hold"};
static const std::string UNHOLD_SERVICE_NAME {"unhold"};

static const std::string OMIT_SERVICE_PARAM_NAME {"omit_service"};
static const std::string STOP1_EXECUTOR_NODE_NAME {"/stop1_executor_node"};

static constexpr double WAIT_FOR_NODE_SLEEPTIME_S {5.0};

/**
 * @brief Test that the startup of the stop1_executor is not complete if a service is missing.
 *
 * @tests{stop1_missing_services,
 *  Test that the startup of the stop1_executor is not complete if a service is missing.
 * }
 *
 * Test Sequence:
 *    1. Advertise all but one manipulator services
 *    2. Advertise missing service from step 1
 *
 * Expected Results:
 *    1. Stop1 executor node does not come up
 *    2. Stop1 executor node does complete startup
 */
TEST(Stop1StartupIntegrationTest, testMissingService)
{
  /**********
   * Setup *
   **********/
  ROS_DEBUG("Test Setup");

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv {"~"};
  std::shared_ptr<ManipulatorMock> manipulator {std::make_shared<ManipulatorMock>()};

  std::map<std::string, std::function<void()>> advertise_funcs;
  advertise_funcs[RECOVER_SERVICE_NAME] = std::bind(&ManipulatorMock::advertiseRecoverService,
                                                   manipulator,
                                                   nh,
                                                   RECOVER_SERVICE_NAME);
  advertise_funcs[HALT_SERVICE_NAME] = std::bind(&ManipulatorMock::advertiseHaltService,
                                                manipulator,
                                                nh,
                                                HALT_SERVICE_NAME);
  advertise_funcs[HOLD_SERVICE_NAME] = std::bind(&ManipulatorMock::advertiseHoldService,
                                                manipulator,
                                                nh,
                                                HOLD_SERVICE_NAME);
  advertise_funcs[UNHOLD_SERVICE_NAME] = std::bind(&ManipulatorMock::advertiseUnholdService,
                                                  manipulator,
                                                  nh,
                                                  UNHOLD_SERVICE_NAME);

  std::string omit_service;
  ASSERT_TRUE(nh_priv.getParam(OMIT_SERVICE_PARAM_NAME, omit_service));

  const auto &it = advertise_funcs.find(omit_service);
  ASSERT_TRUE(it != advertise_funcs.end());

  const auto omit_advertise_func = it->second;
  advertise_funcs.erase(it);

  /**********
   * Step 1 *
   **********/
  ROS_DEBUG("Step 1");

  std::for_each(advertise_funcs.begin(),
                advertise_funcs.end(),
                [](const std::pair<std::string, std::function<void()>> &el){ el.second(); });

  ros::Duration(WAIT_FOR_NODE_SLEEPTIME_S).sleep();
  EXPECT_FALSE(checkForNode(STOP1_EXECUTOR_NODE_NAME));

  /**********
   * Step 2 *
   **********/
  ROS_DEBUG("Step 2");

  omit_advertise_func();
  waitForNode(STOP1_EXECUTOR_NODE_NAME);
}

} // namespace prbt_hardware_support


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "integrationtest_stop1_startup");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{1};
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
