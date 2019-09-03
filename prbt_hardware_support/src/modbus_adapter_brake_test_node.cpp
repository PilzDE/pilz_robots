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

#include <ros/ros.h>

#include <prbt_hardware_support/modbus_adapter_brake_test.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/wait_for_service.h>
#include <prbt_hardware_support/filter_pipeline.h>
#include <prbt_hardware_support/WriteModbusRegister.h>

static const std::string API_SPEC_WRITE_PARAM_NAME("write_api_spec/");
static const std::string SERVICE_NAME_IS_BRAKE_TEST_REQUIRED = "/prbt/brake_test_required";
static const std::string SERVICE_SEND_BRAKE_TEST_RESULT = "/prbt/send_brake_test_result";
static const std::string MODBUS_WRITE_SERVICE_NAME{"/pilz_modbus_client_node/modbus_write"};

using namespace prbt_hardware_support;

static bool writeModbusRegisterCall(ros::ServiceClient& modbus_service,
                                    const uint16_t& start_idx,
                                    const TRegVec& values)
{
  WriteModbusRegister srv;
  srv.request.holding_register_block.start_idx = start_idx;
  srv.request.holding_register_block.values = values;

  ROS_DEBUG_STREAM("Calling service: " << modbus_service.getService() << ")");
  bool call_success = modbus_service.call(srv);
  if (!call_success)
  {
    ROS_ERROR_STREAM("Service call " << modbus_service.getService() << " failed.");
    return false;
  }

  if (!srv.response.success)
  {
    ROS_ERROR_STREAM("Writing of modbus register failed.");
    return false;
  }

  return true;
}


/**
 * @brief Starts a modbus brake test announcer and runs it until a failure occurs.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "modbus_adapter_brake_test");
  ros::NodeHandle nh{};
  ros::NodeHandle pnh{"~"};

  ModbusApiSpec read_api_spec(nh);
  ModbusApiSpec write_api_spec(nh, API_SPEC_WRITE_PARAM_NAME);

  waitForService(MODBUS_WRITE_SERVICE_NAME);
  ROS_DEBUG_STREAM("Done waiting for service: " << MODBUS_WRITE_SERVICE_NAME);
  ros::ServiceClient modbus_write_client = pnh.serviceClient<WriteModbusRegister>(MODBUS_WRITE_SERVICE_NAME);

  using std::placeholders::_1;
  using std::placeholders::_2;
  ModbusAdapterBrakeTest adapter_brake_test(
        std::bind(writeModbusRegisterCall, modbus_write_client, _1, _2),
        read_api_spec, write_api_spec);

  FilterPipeline filter_pipeline(pnh, std::bind(&ModbusAdapterBrakeTest::modbusMsgCallback, &adapter_brake_test, _1) );

  ros::ServiceServer is_brake_test_required_server = pnh.advertiseService(SERVICE_NAME_IS_BRAKE_TEST_REQUIRED,
                                                                           &ModbusAdapterBrakeTest::isBrakeTestRequired,
                                                                           &adapter_brake_test);

  ros::ServiceServer send_brake_test_result = pnh.advertiseService(SERVICE_SEND_BRAKE_TEST_RESULT,
                                                                    &ModbusAdapterBrakeTest::sendBrakeTestResult,
                                                                    &adapter_brake_test);

  ros::spin();

  return EXIT_FAILURE;
}
