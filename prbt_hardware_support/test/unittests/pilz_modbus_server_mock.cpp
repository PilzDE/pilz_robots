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

#include <stdexcept>

#include <prbt_hardware_support/param_names.h>
#include <prbt_hardware_support/pilz_modbus_server_mock.h>

#include <ros/console.h>

namespace prbt_hardware_support
{

PilzModbusServerMock::PilzModbusServerMock(const unsigned int& holding_register_size)
  : holding_register_size_(holding_register_size)
{
  // Create the needed mapping
  // Please note: Only the holding register is used.
  static constexpr unsigned BITS_NB                           {0x00};
  static constexpr unsigned INPUT_BITS_NB                     {0x00};
  static constexpr unsigned int INPUT_REGISTERS_NB            {0x0};
  mb_mapping_ = modbus_mapping_new(BITS_NB, INPUT_BITS_NB, holding_register_size_, INPUT_REGISTERS_NB);
  if (mb_mapping_ == NULL)
  {
    ROS_ERROR("mb_mapping_ is NULL.");
    throw std::runtime_error("mb_mapping_ is NULL.");
  }
}

PilzModbusServerMock::~PilzModbusServerMock()
{
  if (modbus_connection_)
  {
    modbus_close(modbus_connection_);
    modbus_free(modbus_connection_);
  }

  modbus_mapping_free(mb_mapping_);
}

bool PilzModbusServerMock::init(const char *ip, unsigned int port)
{
  modbus_connection_ = modbus_new_tcp(ip, port);
  if(modbus_connection_ == nullptr)
  {
    return false;
  }
  modbus_set_debug(modbus_connection_, true);

  ROS_DEBUG_STREAM("Starting Listening on Port " << ip << ":" << port);
  return true;
}

void PilzModbusServerMock::setHoldingRegister(const std::vector<uint16_t>& data, unsigned int start_index)
{
  if ( data.empty() )
  {
    ROS_ERROR("No data given.");
    return;
  }

  if ( (start_index+data.size()) > holding_register_size_ )
  {
    ROS_ERROR_STREAM("Holding Register is defined from: 0 ... " << holding_register_size_ << "."
                     << " Setting Register " << start_index << " ... " << start_index + data.size() - 1
                     << " is not possible");
    return;
  }

  ROS_DEBUG_STREAM("Set Modbus data for Modbus-Server...");
  modbus_register_access_mutex.lock();
  for (size_t index = 0; index < data.size(); ++index)
  {
    mb_mapping_->tab_registers[start_index+index] = data.at(index);
  }
  modbus_register_access_mutex.unlock();
  ROS_DEBUG_STREAM("Modbus data for Modbus-Server set.");
}

void PilzModbusServerMock::start(const char* ip, const unsigned int port)
{
  init(ip, port);
  run();
}

void PilzModbusServerMock::startAsync(const char* ip, const unsigned int port)
{
  std::unique_lock<std::mutex> lk(running_mutex_);

  thread_ = std::thread{  [this, ip, port]{
    this->start(ip, port);
  }};

  running_cv_.wait(lk);
}

void PilzModbusServerMock::run()
{

#if LIBMODBUS_VERSION_CHECK(3,1,2) // API changed from timeval to uint,uint) in version >= 3.1.2
  modbus_set_response_timeout(modbus_connection_, RESPONSE_TIMEOUT_IN_SEC, RESPONSE_TIMEOUT_IN_USEC);
#else
  struct timeval response_timeout;
  response_timeout.tv_sec = RESPONSE_TIMEOUT_IN_SEC;
  response_timeout.tv_usec = RESPONSE_TIMEOUT_IN_USEC;
  modbus_set_response_timeout(modbus_connection_, &response_timeout);
#endif

  uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

  // Connect to client loop
  while (!terminate_)
  {
    socket_ = modbus_tcp_listen(modbus_connection_, 1);
    // Set socket non blocking
    //fcntl(socket_, F_SETFL, O_NONBLOCK);

{
   std::lock_guard<std::mutex> lk(running_mutex_);
    running_cv_.notify_one();
}

    ROS_ERROR("Notify");

    ROS_DEBUG("Waiting for connection");
    int result {-1};
    while(result < 0)
    {
      ROS_ERROR("Inside Loop");
      result = modbus_tcp_accept(modbus_connection_, &socket_);
      ROS_ERROR("modbus_tcp_accept");

      if(terminate_) break;
    }
    ROS_DEBUG_STREAM("Connection with client accepted.");

    // Loop for reading
    int rc {-1};
    while (!terminate_)
    {
      rc = modbus_receive(modbus_connection_, query);
      if(rc > 0)
      {
        modbus_register_access_mutex.lock();
        modbus_reply(modbus_connection_, query, rc, mb_mapping_);
        modbus_register_access_mutex.unlock();
      }
      else
      {
        ROS_ERROR_STREAM("Connection with client closed");
        break;
      }

      // Allow other threads to run.
      usleep(50);
    } // End reading loop

    close(socket_);
    socket_ = -1;
  } // End connect to client loop
  ROS_DEBUG("Modbus-server run loop finished.");

}

}
