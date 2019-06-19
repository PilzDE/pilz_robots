/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#include <std_srvs/Trigger.h>

#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/param_names.h>
#include <prbt_hardware_support/sto_modbus_adapter.h>
#include <prbt_hardware_support/modbus_msg_sto_wrapper.h>

namespace prbt_hardware_support
{

const std::string PilzStoModbusAdapterNode::HOLD_SERVICE = "manipulator_joint_trajectory_controller/hold";
const std::string PilzStoModbusAdapterNode::UNHOLD_SERVICE = "manipulator_joint_trajectory_controller/unhold";
const std::string PilzStoModbusAdapterNode::RECOVER_SERVICE = "driver/recover";
const std::string PilzStoModbusAdapterNode::HALT_SERVICE = "driver/halt";

PilzStoModbusAdapterNode::PilzStoModbusAdapterNode(ros::NodeHandle &nh, const ModbusApiSpec& api_spec):
  nh_(nh),
  api_spec_(api_spec)
{
  // Wait for services
  while (!ros::service::waitForService(HALT_SERVICE, ros::Duration(RETRY_SERVICE_CONNECTION_TIME_S)) && ros::ok())
  {
    ROS_WARN_STREAM_DELAYED_THROTTLE(WAIT_FOR_SERVICE_TIMEOUT_S, "Waiting for driver service " + HALT_SERVICE
                                     + " to become available... Please check, if the motor driver is up and running.");
  }

  while (!ros::service::waitForService(HOLD_SERVICE, ros::Duration(RETRY_SERVICE_CONNECTION_TIME_S)) && ros::ok())
  {
    ROS_WARN_STREAM_DELAYED_THROTTLE(WAIT_FOR_SERVICE_TIMEOUT_S,
                                     "Waiting for controller service " + HOLD_SERVICE
                                     + " to become available...");
  }

  if(!ros::service::waitForService(UNHOLD_SERVICE, ros::Duration(WAIT_FOR_SERVICE_TIMEOUT_S)))
  {
    ROS_WARN_STREAM("Controller service " << UNHOLD_SERVICE
                    << " not available. STO adapter node can not function properly. You can't unhold the controller"
                       " once it went to hold mode.");
  }
  if(!ros::service::waitForService(RECOVER_SERVICE, ros::Duration(WAIT_FOR_SERVICE_TIMEOUT_S)))
  {
    ROS_WARN_STREAM("Driver service " << RECOVER_SERVICE
                    << " not available. Driver won't auto-recover after STO clearance.");
  }

  // Attach to services (all should be available now)
  halt_srv_client_ = nh.serviceClient<std_srvs::Trigger>(HALT_SERVICE);
  hold_srv_client_ = nh.serviceClient<std_srvs::Trigger>(HOLD_SERVICE);
  unhold_srv_client_ = nh.serviceClient<std_srvs::Trigger>(UNHOLD_SERVICE);
  recover_srv_client_ = nh.serviceClient<std_srvs::Trigger>(RECOVER_SERVICE);

  modbus_sub_ = std::make_shared< message_filters::Subscriber<ModbusMsgInStamped> >(nh, TOPIC_MODBUS_READ, 1);
  update_filter_ = std::make_shared< message_filters::UpdateFilter<ModbusMsgInStamped> >(*modbus_sub_);

  // Register to modbus callback
  update_filter_->registerCallback(boost::bind(&PilzStoModbusAdapterNode::modbusInMsgCallback, this, _1));
}

PilzStoModbusAdapterNode::~PilzStoModbusAdapterNode()
{
  hold_srv_client_.shutdown();
  halt_srv_client_.shutdown();
  unhold_srv_client_.shutdown();
  recover_srv_client_.shutdown();
}

void PilzStoModbusAdapterNode::performStop()
{
  ROS_INFO("Performing STOP due to STO signal");

  std_srvs::Trigger trigger;

  ROS_DEBUG_STREAM("Calling Hold on controller (Service: " << hold_srv_client_.getService() << ")");
  hold_srv_client_.call(trigger);
  ros::Duration(DURATION_BETWEEN_HOLD_AND_DISABLE_MS/1000.0).sleep();

  ROS_DEBUG_STREAM("Calling Halt on driver (Service: " << halt_srv_client_.getService() << ")");
  halt_srv_client_.call(trigger);
}

void PilzStoModbusAdapterNode::modbusInMsgCallback(const ModbusMsgInStampedConstPtr& msg_raw)
{
  try
  {
    ModbusMsgStoWrapper msg(msg_raw, api_spec_);
    internalMsgCallback(msg);
  }
  catch(const ModbusMsgWrapperException &e)
  {
    performStop();
    ROS_ERROR_STREAM(e.what());
    return;
  }
}

void PilzStoModbusAdapterNode::internalMsgCallback(const ModbusMsgStoWrapper& msg)
{
  if(msg.isDisconnect())
  {
    ROS_ERROR("A disconnect from the modbus server happend.");
    performStop();

    return;
  }

  int modbus_api_version = msg.getVersion();
  if(modbus_api_version != MODBUS_API_VERSION_REQUIRED)
  {
    ROS_ERROR_STREAM("Received Modbus Message with unsupported API Version "
                    << modbus_api_version << ", required Version is " << MODBUS_API_VERSION_REQUIRED);
    performStop();
    return;
  }

  if(!msg.getSTO())
  {
    performStop();
  }
  else
  {
    std_srvs::Trigger trigger;

    if (unhold_srv_client_.exists())
    {
      ROS_DEBUG_STREAM("Calling Unhold (Service: " << unhold_srv_client_.getService() << ")");
      unhold_srv_client_.call(trigger);
    }
    else
    {
      ROS_WARN_STREAM("Calling Unhold (Service: " << unhold_srv_client_.getService() << ") failed.");
    }
    if (recover_srv_client_.exists())
    {
      ROS_DEBUG_STREAM("Calling Recover (Service: " << recover_srv_client_.getService() << ")");
      recover_srv_client_.call(trigger);
    }
    else
    {
      ROS_WARN_STREAM("Calling Recover (Service: " << recover_srv_client_.getService() << ") failed.");
    }
  }
}

}  // prbt_hardware_support
