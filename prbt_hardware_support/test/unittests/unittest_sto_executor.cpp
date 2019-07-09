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

#include <functional>
#include <memory>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <prbt_hardware_support/sto_executor.h>
#include <prbt_hardware_support/service_client_mock.h>

namespace prbt_hardware_support_tests
{

using namespace prbt_hardware_support;

typedef ServiceClientMockFactory<std_srvs::Trigger> MockFactory;
typedef STOExecutorTemplated<ServiceClientMock<std_srvs::Trigger>> STOExecutor;

/**
 * @brief Test D0 destructor
 *
 * Increases function coverage
 */
TEST(STOExecutorTest, testD0estructor)
{
  MockFactory factory;
  std::shared_ptr<STOExecutor> sto_executor {new STOExecutor(std::bind(&MockFactory::create, &factory,
                                                                       std::placeholders::_1))};
}

/**
 * @brief destructor ends run
 */

/**
 * @brief terminate ends run
 */

/**
 * @brief enable
 *
 * Test Sequence:
 *  1. update(true)
 *
 * Expected Results:
 *  1. recover, unhold
 */

/**
 * @brief disable plus recover
 *
 * Test Sequence:
 *  1. update(true), update(false) during unhold, update(true) during halt
 *
 * Expected Results:
 *  1. recover, unhold, hold, halt, recover, unhold
 */

/**
 * @brief spam enable
 *
 * Test Sequence:
 *  1. update(true) periodically
 *
 * Expected Results:
 *  1. recover, unhold
 */

/**
 * @brief spam disable plus recover
 *
 * Test Sequence:
 *  1. update(true), update(false) during unhold and periodically, update(true)
 *
 * Expected Results:
 *  1. recover, unhold, hold, halt, recover, unhold
 */

/**
 * @brief skip hold
 *
 * Test Sequence:
 *  1. update(true), update(false) during recover, update(true) during halt
 *
 * Expected Results:
 *  1. recover, halt, recover, unhold
 */

/**
 * @brief skip halt
 *
 * Test Sequence:
 *  1. update(true), update(false) during unhold, update(true) during hold, update(false) during unhold
 *
 * Expected Results:
 *  1. recover, unhold, hold, unhold, hold, halt
 */

/**
 * @brief recover fail
 *
 * Test Sequence:
 *  1. update(true), recover fails, update(false), update(true)
 *
 * Expected Results:
 *  1. recover, halt, recover, unhold
 */

/**
 * @brief unhold fail
 *
 * Test Sequence:
 *  1. update(true), unhold fails, update(false), update(true) during halt
 *
 * Expected Results:
 *  1. recover, unhold, hold, halt, recover, unhold
 */

/**
 * @brief hold fail
 *
 * Test Sequence:
 *  1. update(true), update(false) during unhold, hold fails
 *
 * Expected Results:
 *  1. recover, unhold, hold, halt
 */

/**
 * @brief halt fail
 *
 * Test Sequence:
 *  1. update(true), update(false) during unhold, halt fails
 *
 * Expected Results:
 *  1. recover, unhold, hold, halt
 */

}  // namespace prbt_hardware_support_tests

int main(int argc, char **argv){
  // for (limited) ros::Time functionality, no ROS communication
  ros::Time::init();

  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}