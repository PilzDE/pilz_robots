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

#include <prbt_hardware_support/modbus_msg_brake_test_wrapper.h>

#include <functional>
#include <sstream>

namespace prbt_hardware_support
{

static const std::string SERVICE_NAME_IS_BRAKE_TEST_REQUIRED = "/prbt/brake_test_required";

static constexpr unsigned int MODBUS_API_VERSION_REQUIRED {2};

using std::placeholders::_1;

ModbusAdapterBrakeTest::ModbusAdapterBrakeTest(ros::NodeHandle& nh, const ModbusApiSpec& api_spec)
  : api_spec_(api_spec)
  , filter_pipeline_(new FilterPipeline(nh, std::bind(&ModbusAdapterBrakeTest::modbusMsgCallback, this, _1 )) )
{
  is_brake_test_required_server_ = nh.advertiseService(SERVICE_NAME_IS_BRAKE_TEST_REQUIRED,
                                                      &ModbusAdapterBrakeTest::isBrakeTestRequired,
                                                      this);
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

void ModbusAdapterBrakeTest::updateBrakeTestRequiredState(IsBrakeTestRequiredResponse::_result_type brake_test_required)
{
  TBrakeTestRequired last_brake_test_flag {brake_test_required_};
  brake_test_required_ = brake_test_required;
  if(brake_test_required_ == IsBrakeTestRequiredResponse::REQUIRED
     && last_brake_test_flag != IsBrakeTestRequiredResponse::REQUIRED)
  {
    ROS_INFO("Brake Test required.");
  }
}

bool ModbusAdapterBrakeTest::isBrakeTestRequired(IsBrakeTestRequired::Request& /*req*/,
                                                 IsBrakeTestRequired::Response& res)
{
  res.result = brake_test_required_;
  return true;
}

} // namespace prbt_hardware_support
