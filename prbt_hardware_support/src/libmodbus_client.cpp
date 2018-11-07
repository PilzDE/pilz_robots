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

#include <ros/ros.h>

#include <vector>
#include <errno.h>

#include <prbt_hardware_support/libmodbus_client.h>
#include <prbt_hardware_support/pilz_modbus_exceptions.h>

namespace prbt_hardware_support
{

LibModbusClient::~LibModbusClient()
{
  close();
}

bool LibModbusClient::init(const char* ip, unsigned int port)
{
  modbus_connection_ = modbus_new_tcp(ip, port);

  if (modbus_connect(modbus_connection_) == -1)
  {
    ROS_ERROR_STREAM("Could not establish modbus connection." << modbus_strerror(errno));
    modbus_free(modbus_connection_);
    modbus_connection_ = nullptr;
    return false;
  }
  return true;
}

void LibModbusClient::setResponseTimeoutInMs(unsigned int timeout_ms)
{
  struct timeval response_timeout;
  response_timeout.tv_sec = timeout_ms/1000;
  response_timeout.tv_usec = (timeout_ms % 1000) * 1000;
  modbus_set_response_timeout(modbus_connection_, &response_timeout);
}

unsigned int LibModbusClient::getResponseTimeoutInMs()
{
  struct timeval response_timeout;
  modbus_get_response_timeout(modbus_connection_, &response_timeout);
  return response_timeout.tv_sec * 1000 + (response_timeout.tv_usec  / 1000);
}

std::vector<uint16_t> LibModbusClient::readHoldingRegister(int addr, int nb)
{
  if(modbus_connection_ == nullptr)
  {
    throw ModbusExceptionDisconnect("Modbus disconnected!");
  }

  uint16_t tab_reg[nb];
  int rc {-1};

  rc = modbus_read_registers(modbus_connection_, addr, nb, tab_reg);
  if (rc == -1)
  {
    throw ModbusExceptionDisconnect("Modbus disconnected!");
  }

  return std::vector<uint16_t> (tab_reg, tab_reg + nb);

}

void LibModbusClient::close()
{
  if (modbus_connection_)
  {
    modbus_close(modbus_connection_);
    modbus_free(modbus_connection_);
    modbus_connection_ = nullptr;
  }
}

}
