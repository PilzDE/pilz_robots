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
#include <prbt_hardware_support/modbus_adapter_brake_test.h>

#include <functional>
#include <sstream>
#include <algorithm>

#include <prbt_hardware_support/modbus_msg_brake_test_wrapper.h>
#include <prbt_hardware_support/wait_for_service.h>
#include <prbt_hardware_support/WriteModbusRegister.h>
#include <prbt_hardware_support/modbus_adapter_brake_test_exception.h>

namespace prbt_hardware_support
{

static const std::string SERVICE_NAME_IS_BRAKE_TEST_REQUIRED = "/prbt/brake_test_required";
static const std::string SERVICE_SEND_BRAKE_TEST_RESULT = "/prbt/send_brake_test_result";
static const std::string MODBUS_WRITE_SERVICE_NAME{"/pilz_modbus_client_node/modbus_write"};

static constexpr unsigned int MODBUS_API_VERSION_REQUIRED {2};

using std::placeholders::_1;

ModbusAdapterBrakeTest::ModbusAdapterBrakeTest(ros::NodeHandle& nh,
                                               const ModbusApiSpec& read_api_spec,
                                               const ModbusApiSpec& write_api_spec)
  : api_spec_(read_api_spec)
  , filter_pipeline_(new FilterPipeline(nh, std::bind(&ModbusAdapterBrakeTest::modbusMsgCallback, this, _1 )) )
  , reg_idx_cont_(getRegisters(write_api_spec))
  , reg_start_idx_(getMinRegisterIdx(reg_idx_cont_))
  , reg_block_size_(getRegisterBlockSize(reg_idx_cont_))
{
  is_brake_test_required_server_ = nh.advertiseService(SERVICE_NAME_IS_BRAKE_TEST_REQUIRED,
                                                       &ModbusAdapterBrakeTest::isBrakeTestRequired,
                                                       this);

  send_brake_test_result_ = nh.advertiseService(SERVICE_SEND_BRAKE_TEST_RESULT,
                                                &ModbusAdapterBrakeTest::sendBrakeTestResult,
                                                this);

  waitForService(MODBUS_WRITE_SERVICE_NAME);
  ROS_DEBUG_STREAM("Done waiting for service: " << MODBUS_WRITE_SERVICE_NAME);
  modbus_write_client_ = nh.serviceClient<WriteModbusRegister>(MODBUS_WRITE_SERVICE_NAME);
}

ModbusAdapterBrakeTest::TRegIdxCont ModbusAdapterBrakeTest::getRegisters(const ModbusApiSpec &write_api_spec)
{
  TRegIdxCont reg_idx_cont;

  if(!write_api_spec.hasRegisterDefinition(modbus_api_spec::BRAKETEST_PERFORMED))
  {
    throw ModbusAdapterBrakeTestException("Failed to read API spec for BRAKETEST_PERFORMED");
  }
  reg_idx_cont[modbus_api_spec::BRAKETEST_PERFORMED] =
                       write_api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_PERFORMED);

  if(!write_api_spec.hasRegisterDefinition(modbus_api_spec::BRAKETEST_RESULT))
  {
    throw ModbusAdapterBrakeTestException("Failed to read API spec for BRAKETEST_RESULT");
  }
  reg_idx_cont[modbus_api_spec::BRAKETEST_RESULT] =
                       write_api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_RESULT);

  if(abs(reg_idx_cont.at(modbus_api_spec::BRAKETEST_PERFORMED) - reg_idx_cont.at(modbus_api_spec::BRAKETEST_RESULT)) != 1)
  {
    std::ostringstream os;
    os << "Registers of BRAKETEST_PERFORMED and BRAKETEST_RESULT need to be 1 apart";
    os << " (distance: " << abs(reg_idx_cont.cbegin()->second - reg_idx_cont.cend()->second) << ")";
    // Both registers need to be one apart, so that we can write them in one cycle
    throw ModbusAdapterBrakeTestException(os.str());
  }

  return reg_idx_cont;
}

void ModbusAdapterBrakeTest::modbusMsgCallback(const ModbusMsgInStampedConstPtr& msg_raw)
{
  ModbusMsgBrakeTestWrapper msg {msg_raw, api_spec_};

  if (msg.isDisconnect())
  {
    return;
  }

  try
  {
    msg.checkStructuralIntegrity();
  }
  catch(const ModbusMsgWrapperException &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return;
  }

  if(msg.getVersion() != MODBUS_API_VERSION_REQUIRED)
  {
    std::ostringstream os;
    os << "Received Modbus message of unsupported API Version: "
       << msg.getVersion()
       << ", required Version: " << MODBUS_API_VERSION_REQUIRED;
    os <<"\n";
    os << "Can not determine from Modbus message if brake-test is required.";
    ROS_ERROR_STREAM(os.str());
    return;
  }

  updateBrakeTestRequiredState(msg.getBrakeTestRequirementStatus());
}

void ModbusAdapterBrakeTest::updateBrakeTestRequiredState(TBrakeTestRequired brake_test_required)
{
  TBrakeTestRequired last_brake_test_flag {brake_test_required_};
  brake_test_required_ = brake_test_required;
  if(brake_test_required_ == IsBrakeTestRequiredResponse::REQUIRED
     && last_brake_test_flag != IsBrakeTestRequiredResponse::REQUIRED)
  {
    ROS_INFO("Brake Test required.");
  }
}

bool ModbusAdapterBrakeTest::sendBrakeTestResult(SendBrakeTestResult::Request& req,
                                                 SendBrakeTestResult::Response& res)
{
  TRegVec reg_cont(reg_block_size_, 0);
  reg_cont.at(reg_idx_cont_.at(modbus_api_spec::BRAKETEST_PERFORMED) - reg_start_idx_) = req.performed;
  reg_cont.at(reg_idx_cont_.at(modbus_api_spec::BRAKETEST_RESULT) - reg_start_idx_) = req.result;

  WriteModbusRegister srv;
  srv.request.holding_register_block.start_idx = reg_start_idx_;
  srv.request.holding_register_block.values = reg_cont;

  ROS_DEBUG_STREAM("Calling service: " << modbus_write_client_.getService() << ")");
  bool call_success = modbus_write_client_.call(srv);
  if (!call_success)
  {
    std::ostringstream os;
    os << "No success calling service: " << modbus_write_client_.getService();
    res.error_msg = os.str();
  }

  if (!srv.response.success)
  {
    res.error_msg = "Failed to send brake test result to FS control";
  }

  res.success = call_success && srv.response.success;
  return true;
}

} // namespace prbt_hardware_support
