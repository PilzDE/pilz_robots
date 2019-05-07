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

#include <string>

#include <ros/ros.h>

#include <prbt_hardware_support/BrakeTest.h>

#include <prbt_hardware_support/brake_test_executor.h>
#include <prbt_hardware_support/brake_test_executor_exception.h>
#include <prbt_hardware_support/brake_test_utils.h>

namespace prbt_hardware_support
{

static constexpr double WAIT_FOR_SERVICE_TIMEOUT_S{5.0};

static const std::string EXECUTE_BRAKETEST_SERVICE_NAME{"/prbt/execute_braketest"};
static const std::string BRAKETEST_ADAPTER_SERVICE_NAME{"/prbt/braketest_adapter_node/trigger_braketest"};

BrakeTestExecutor::BrakeTestExecutor(ros::NodeHandle& nh)
  :nh_(nh)
{
  brake_test_srv_ = nh_.advertiseService(EXECUTE_BRAKETEST_SERVICE_NAME,
                                         &BrakeTestExecutor::executeBrakeTest,
                                         this);

  trigger_braketest_client_ = nh_.serviceClient<BrakeTest>(BRAKETEST_ADAPTER_SERVICE_NAME);

  if (!trigger_braketest_client_.waitForExistence(ros::Duration(WAIT_FOR_SERVICE_TIMEOUT_S)))
  {
    throw BrakeTestExecutorException("Service " + trigger_braketest_client_.getService() + " not available.");
  }
}

bool BrakeTestExecutor::executeBrakeTest(BrakeTest::Request&,
                                         BrakeTest::Response& response)
{
  if (BrakeTestUtils::detectRobotMotion())
  {
    response.success = false;
    response.error_msg = "Robot is moving, cannot perform brake test";
    response.error_code.value = BrakeTestErrorCodes::ROBOT_MOTION_DETECTED;
    return true;
  }

  BrakeTest srv;
  if (!trigger_braketest_client_.call(srv))
  {
    response.success = false;
    response.error_msg = "Failed to trigger brake test via service " + trigger_braketest_client_.getService();
    response.error_code.value = BrakeTestErrorCodes::TRIGGER_BRAKETEST_SERVICE_FAILURE;
    return true;
  }

  response = srv.response;
  return true;
}

}
