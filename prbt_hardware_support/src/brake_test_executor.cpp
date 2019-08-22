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

#include <std_srvs/Trigger.h>

#include <prbt_hardware_support/BrakeTest.h>
#include <prbt_hardware_support/ModbusRegisterBlock.h>
#include <prbt_hardware_support/WriteModbusRegister.h>
#include <prbt_hardware_support/WriteModbusRegisterRequest.h>
#include <prbt_hardware_support/WriteModbusRegisterResponse.h>

#include <prbt_hardware_support/brake_test_executor.h>
#include <prbt_hardware_support/brake_test_executor_exception.h>
#include <prbt_hardware_support/brake_test_utils.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/wait_for_service.h>

namespace prbt_hardware_support
{

static const std::string EXECUTE_BRAKETEST_SERVICE_NAME{"/prbt/execute_braketest"};
static const std::string BRAKETEST_ADAPTER_SERVICE_NAME{"/prbt/braketest_adapter_node/trigger_braketest"};

static const std::string CONTROLLER_HOLD_MODE_SERVICE_NAME{"/prbt/manipulator_joint_trajectory_controller/hold"};
static const std::string CONTROLLER_UNHOLD_MODE_SERVICE_NAME{"/prbt/manipulator_joint_trajectory_controller/unhold"};
static const std::string MODBUS_WRITE_SERVICE_NAME{"/pilz_modbus_client_node/modbus_write"};

static const std::string API_SPEC_READ_PARAM_NAME("write_api_spec/");

BrakeTestExecutor::BrakeTestExecutor(ros::NodeHandle& nh)
  :nh_(nh)
{
  brake_test_srv_ = nh_.advertiseService(EXECUTE_BRAKETEST_SERVICE_NAME,
                                         &BrakeTestExecutor::executeBrakeTest,
                                         this);

  // set up braketest service client (required)
  waitForService(BRAKETEST_ADAPTER_SERVICE_NAME);
  trigger_braketest_client_ = nh_.serviceClient<BrakeTest>(BRAKETEST_ADAPTER_SERVICE_NAME);

  // set up hold service client
  waitForService(CONTROLLER_HOLD_MODE_SERVICE_NAME);
  controller_hold_client_ = nh_.serviceClient<std_srvs::Trigger>(CONTROLLER_HOLD_MODE_SERVICE_NAME);

  // set up unhold service client
  waitForService(CONTROLLER_UNHOLD_MODE_SERVICE_NAME);
  controller_unhold_client_ = nh_.serviceClient<std_srvs::Trigger>(CONTROLLER_UNHOLD_MODE_SERVICE_NAME);

  // set up modbus write service client
  waitForService(MODBUS_WRITE_SERVICE_NAME);
  modbus_write_client_ = nh_.serviceClient<WriteModbusRegister>(MODBUS_WRITE_SERVICE_NAME);

  // get brake test result register numbers
  ModbusApiSpec read_api_spec {nh_, API_SPEC_READ_PARAM_NAME};
  if(!read_api_spec.hasRegisterDefinition(modbus_api_spec::BRAKETEST_PERFORMED))
  {
    throw BrakeTestExecutorException("failed to read API spec for BRAKETEST_PERFORMED");
  }
  short unsigned int brake_test_performed_modbus_register = static_cast<short unsigned int>(read_api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_PERFORMED));
  if(!read_api_spec.hasRegisterDefinition(modbus_api_spec::BRAKETEST_RESULT))
  {
    throw BrakeTestExecutorException("failed to read API spec for BRAKETEST_RESULT");
  }
  short unsigned int brake_test_result_modbus_register = static_cast<short unsigned int>(read_api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_RESULT));
  if(abs(brake_test_performed_modbus_register - brake_test_result_modbus_register) != 1)
  {
    // Both registers need to be one apart, so that we can write them in one cycle
    throw BrakeTestExecutorException("registers of BRAKETEST_PERFORMED and BRAKETEST_RESULT need to be 1 apart");
  }
  // starting from the lowest register of the two
  brake_test_modbus_register_low_ = std::min(brake_test_performed_modbus_register, brake_test_result_modbus_register);
}

bool BrakeTestExecutor::executeBrakeTest(BrakeTest::Request& /*req*/,
                                         BrakeTest::Response& response)
{
  if (BrakeTestUtils::detectRobotMotion())
  {
    response.success = false;
    response.error_msg = "Robot is moving, cannot perform brake test";
    response.error_code.value = BrakeTestErrorCodes::ROBOT_MOTION_DETECTED;
    return true;
  }

  ROS_INFO_STREAM("Enter hold for braketest. Do not unhold the controller!");

  std_srvs::Trigger trigger_srv;
  if (!controller_hold_client_.call(trigger_srv))
  {
    ROS_WARN_STREAM("Failed to trigger hold via service " << controller_hold_client_.getService());
  }

  BrakeTest braketest_srv;
  if (!trigger_braketest_client_.call(braketest_srv))
  {
    response.success = false;
    response.error_msg = "Failed to trigger brake test via service " + trigger_braketest_client_.getService();
    response.error_code.value = BrakeTestErrorCodes::TRIGGER_BRAKETEST_SERVICE_FAILURE;
  }
  else
  {
    response = braketest_srv.response;
  }

  if (!controller_unhold_client_.call(trigger_srv))
  {
    ROS_WARN_STREAM("Failed to trigger unhold via service " << controller_unhold_client_.getService());
  }

  if(response.success)
  {
    WriteModbusRegister srv;
    srv.request.holding_register_block.start_idx = brake_test_modbus_register_low_;
    std::vector<unsigned short> values_to_set = {0, 1}; // Note: The FS controller needs a positive edge, so we first send 0s.
    for(const auto &v : values_to_set)
    {
      srv.request.holding_register_block.values = {v, v};
      modbus_write_client_.call(srv);
      if(!srv.response.success){
        ROS_ERROR_STREAM("Failed to send brake test result to FS control");
      }
    }
  }

  return true;
}

} // namespace prbt_hardware_support
