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

#include <prbt_hardware_support/pilz_modbus_read_client.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_msg_in_utils.h>
#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/pilz_modbus_exceptions.h>
#include <prbt_hardware_support/pilz_modbus_read_client_exception.h>

namespace prbt_hardware_support
{
    
PilzModbusReadClient::PilzModbusReadClient(ros::NodeHandle& nh,
                                           const unsigned int num_registers_to_read,
                                           const unsigned int index_of_first_register,
                                           ModbusClientUniquePtr modbus_client)
  : NUM_REGISTERS_TO_READ(num_registers_to_read)
  , INDEX_OF_FIRST_REGISTER(index_of_first_register)
  , modbus_client_(std::move(modbus_client))
  , modbus_pub_(nh.advertise<ModbusMsgInStamped>(TOPIC_MODBUS_READ, DEFAULT_QUEUE_SIZE_MODBUS))
{
}


bool PilzModbusReadClient::init(const char* ip, unsigned int port,
                                              unsigned int retries, ros::Duration timeout)
{
  for(size_t retry_n = 0; retry_n < retries; ++retry_n)
  {
    if(init(ip, port))
    {
      return true;
    }

    ROS_ERROR_STREAM("Connection to " << ip << ":" << port << " failed. Try(" << retry_n+1 << "/" << retries << ")");

    if(!ros::ok())
    {
      break; // LCOV_EXCL_LINE Simple functionality but hard to test
    }
    timeout.sleep();
  }

  return false;
}


bool PilzModbusReadClient::init(const char* ip, unsigned int port)
{
  State expectedState {State::not_initialized};
  if (!state_.compare_exchange_strong(expectedState, State::initializing))
  {
    ROS_ERROR_STREAM("Modbus-client not in correct state." << state_);
    state_ = State::not_initialized;
    return false;
  }

  if (!modbus_client_->init(ip, port))
  {
    ROS_ERROR("Init failed");
    state_ = State::not_initialized;
    return false;
  }

  modbus_client_->setResponseTimeoutInMs(RESPONSE_TIMEOUT_IN_MS);

  state_ = State::initialized;
  ROS_DEBUG_STREAM("Connection to " << ip << ":" << port << " establised");
  return true;
}


void PilzModbusReadClient::sendDisconnectMsg()
{
  ModbusMsgInStamped msg;
  msg.disconnect.data = true;
  msg.header.stamp = ros::Time::now();
  modbus_pub_.publish(msg);
  ros::spinOnce();
}


void PilzModbusReadClient::run()
{
  State expectedState {State::initialized};
  if (!state_.compare_exchange_strong(expectedState, State::running))
  {
    throw PilzModbusReadClientException("Modbus-client not in correct state.");
  }

  std::vector<uint16_t> holding_register;
  std::vector<uint16_t> last_holding_register;
  ros::Time last_update {ros::Time::now()};
  state_ = State::running;
  ros::Rate rate(MODBUS_RATE_HZ);
  while ( ros::ok() && !stop_run_.load() )
  {
    try
    {
      holding_register = modbus_client_->readHoldingRegister(INDEX_OF_FIRST_REGISTER, NUM_REGISTERS_TO_READ);
    }
    catch(ModbusExceptionDisconnect &e)
    {
      ROS_ERROR_STREAM(e.what());
      sendDisconnectMsg();
      break;
    }

    ModbusMsgInStampedPtr msg {createDefaultModbusMsgIn(INDEX_OF_FIRST_REGISTER, holding_register)};

    // Publish the received data into ROS
    if(holding_register != last_holding_register)
    {
      ROS_DEBUG_STREAM("Sending new ROS-message.");
      msg->header.stamp = ros::Time::now();
      last_update = msg->header.stamp;
      last_holding_register = holding_register;
    }
    else
    {
      msg->header.stamp = last_update;
    }
    modbus_pub_.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  stop_run_ = false;
  state_ = State::not_initialized;
}

}  // namespace prbt_hardware_support