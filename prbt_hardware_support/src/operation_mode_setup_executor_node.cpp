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

#include <prbt_hardware_support/operation_mode_setup_executor.h>
#include <prbt_hardware_support/wait_for_service.h>
#include <prbt_hardware_support/SetSpeedLimit.h>
#include <prbt_hardware_support/set_speed_limit_func_decl.h>
#include <prbt_hardware_support/GetOperationMode.h>
#include <prbt_hardware_support/get_operation_mode_func_decl.h>
#include <prbt_hardware_support/get_param.h>

static const std::string SET_SPEED_LIMIT_SERVICE{"set_speed_limit"};
static const std::string OPERATION_MODE_SERVICE{"/prbt/get_operation_mode"};
static const std::string OPERATION_MODE_TOPIC{"operation_mode"};

static const std::string PARAM_SPEED_LIMIT_T1_STR {"speed_limit_t1"};
static const std::string PARAM_SPEED_LIMIT_AUTO_STR {"speed_limit_automatic"};

static constexpr uint32_t DEFAULT_QUEUE_SIZE {10} ;

// LCOV_EXCL_START
namespace prbt_hardware_support
{

bool setSpeedLimitSrv(ros::ServiceClient& srv_client, const double& speed_limit)
{
  SetSpeedLimit srv_msg;
  srv_msg.request.speed_limit = speed_limit;
  bool call_success = srv_client.call(srv_msg);
  if (!call_success)
  {
    ROS_ERROR_STREAM("No success calling service: " << srv_client.getService());
  }
  return call_success;
}

OperationModes getOperationMode(ros::ServiceClient& srv_client)
{
  GetOperationMode srv_msg;
  bool call_success = srv_client.call(srv_msg);
  if (!call_success)
  {
    ROS_ERROR_STREAM("No success calling service: " << srv_client.getService());
  }
  return srv_msg.response.mode;
}

} // namespace prbt_hardware_support

using namespace prbt_hardware_support;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "operation_mode_setup_executor");
  ros::NodeHandle nh;

  using std::placeholders::_1;
  waitForService(SET_SPEED_LIMIT_SERVICE);
  ros::ServiceClient speed_limit_srv = nh.serviceClient<SetSpeedLimit>(SET_SPEED_LIMIT_SERVICE);
  TSetSpeedLimit set_speed_limit_func = std::bind(setSpeedLimitSrv, speed_limit_srv, _1);

  waitForService(OPERATION_MODE_SERVICE);
  ros::ServiceClient op_mode_srv = nh.serviceClient<GetOperationMode>(OPERATION_MODE_SERVICE);
  TGetOpMode get_op_mode_func = std::bind(getOperationMode, op_mode_srv);

  double speed_limit_t1 {0};
  double speed_limit_auto {0};
  ros::NodeHandle pnh{"~"};
  try
  {
    speed_limit_t1 = getParam<double>(pnh, PARAM_SPEED_LIMIT_T1_STR);
    speed_limit_auto = getParam<double>(pnh, PARAM_SPEED_LIMIT_AUTO_STR);
  }
  catch (const std::runtime_error &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return EXIT_FAILURE;
  }

  OperationModeSetupExecutor op_mode_executor(speed_limit_t1, speed_limit_auto,
                                              set_speed_limit_func,
                                              get_op_mode_func);

  ros::Subscriber operation_mode_sub = nh.subscribe(OPERATION_MODE_TOPIC, DEFAULT_QUEUE_SIZE,
                                            &OperationModeSetupExecutor::updateOperationMode,
                                            &op_mode_executor);
  ros::spin();

  return EXIT_FAILURE;
}
// LCOV_EXCL_STOP
