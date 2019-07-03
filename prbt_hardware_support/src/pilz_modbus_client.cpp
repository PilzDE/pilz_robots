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

#include <prbt_hardware_support/pilz_modbus_client.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_msg_in_utils.h>
#include <prbt_hardware_support/pilz_modbus_exceptions.h>
#include <prbt_hardware_support/pilz_modbus_client_exception.h>

namespace prbt_hardware_support
{

PilzModbusClient::PilzModbusClient(ros::NodeHandle& nh,
                                   const unsigned int num_registers_to_read,
                                   const unsigned int index_of_first_register,
                                   ModbusClientUniquePtr modbus_client,
                                   unsigned int response_timeout_ms,
                                   const std::string& modbus_read_topic_name,
                                   const std::string& modbus_write_service_name,
                                   double read_frequency_hz)
  : NUM_REGISTERS_TO_READ(num_registers_to_read)
  , INDEX_OF_FIRST_REGISTER(index_of_first_register)
  , RESPONSE_TIMEOUT_MS(response_timeout_ms)
  , READ_FREQUENCY_HZ(read_frequency_hz)
  , modbus_client_(std::move(modbus_client))
  , modbus_read_pub_(nh.advertise<ModbusMsgInStamped>(modbus_read_topic_name, DEFAULT_QUEUE_SIZE_MODBUS))
  , modbus_write_service_( nh.advertiseService(modbus_write_service_name,
                                               &PilzModbusClient::modbus_write_service_cb,
                                               this) )
{
}


bool PilzModbusClient::init(const char* ip, unsigned int port,
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


bool PilzModbusClient::init(const char* ip, unsigned int port)
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

  modbus_client_->setResponseTimeoutInMs(RESPONSE_TIMEOUT_MS);

  state_ = State::initialized;
  ROS_DEBUG_STREAM("Connection to " << ip << ":" << port << " establised");
  return true;
}

void PilzModbusClient::sendDisconnectMsg()
{
  ModbusMsgInStamped msg;
  msg.disconnect.data = true;
  msg.header.stamp = ros::Time::now();
  modbus_read_pub_.publish(msg);
  ros::spinOnce();
}

void PilzModbusClient::run()
{
  State expectedState {State::initialized};
  if (!state_.compare_exchange_strong(expectedState, State::running))
  {
    throw PilzModbusClientException("Modbus-client not in correct state.");
  }

  RegCont holding_register;
  RegCont last_holding_register;
  ros::Time last_update {ros::Time::now()};
  state_ = State::running;
  ros::Rate rate(READ_FREQUENCY_HZ);
  while ( ros::ok() && !stop_run_.load() )
  {
    // Work with local copy of buffer to ensure that the service callback
    // function does not become blocked
    boost::optional<ModbusRegisterBlock> write_reg_bock {boost::none};
    {
      std::lock_guard<std::mutex> lock(write_reg_blocks_mutex_);
      if (!write_reg_blocks_.empty())
      {
        write_reg_bock = write_reg_blocks_.front();
        // Mark data as send/processed, by "deleting" them from memory
        write_reg_blocks_.pop();
      }
    }

    try
    {
      if (write_reg_bock)
      {

        holding_register = modbus_client_->writeReadHoldingRegister(static_cast<int>(write_reg_bock->start_idx),
                                                                    write_reg_bock->values,
                                                                    static_cast<int>(INDEX_OF_FIRST_REGISTER),
                                                                    static_cast<int>(NUM_REGISTERS_TO_READ));

      }
      else
      {
        holding_register = modbus_client_->readHoldingRegister(static_cast<int>(INDEX_OF_FIRST_REGISTER), static_cast<int>(NUM_REGISTERS_TO_READ));
      }
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
    modbus_read_pub_.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  stop_run_ = false;
  state_ = State::not_initialized;
}

}  // namespace prbt_hardware_support
