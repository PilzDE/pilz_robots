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

#include <cstddef>
#include <vector>
#include <errno.h>
#include <limits>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <arpa/inet.h>
#include <fcntl.h>
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
  if (!checkIPConnection(ip, port))
  {
    ROS_DEBUG_STREAM("Precheck for connection to " << ip << ":" << port << " failed.");
    return false;
  }

  modbus_connection_ = modbus_new_tcp(ip, static_cast<int>(port));

  if (modbus_connect(modbus_connection_) == -1)
  {
    ROS_ERROR_STREAM_NAMED("LibModbusClient", "Could not establish modbus connection." << modbus_strerror(errno));
    modbus_free(modbus_connection_);
    modbus_connection_ = nullptr;
    return false;
  }

  return true;
}

void LibModbusClient::setResponseTimeoutInMs(unsigned long timeout_ms)
{
  struct timeval response_timeout;
  response_timeout.tv_sec = timeout_ms / 1000;
  response_timeout.tv_usec = (timeout_ms % 1000) * 1000;
  modbus_set_response_timeout(modbus_connection_, &response_timeout);
}

unsigned long LibModbusClient::getResponseTimeoutInMs()
{
  struct timeval response_timeout;
  modbus_get_response_timeout(modbus_connection_, &response_timeout);
  return static_cast<unsigned long>(response_timeout.tv_sec * 1000L + (response_timeout.tv_usec / 1000L));
}

RegCont LibModbusClient::readHoldingRegister(int addr, int nb)
{
  ROS_DEBUG("readHoldingRegister()");
  if (modbus_connection_ == nullptr)
  {
    throw ModbusExceptionDisconnect("Modbus disconnected!");
  }

  RegCont tab_reg(static_cast<RegCont::size_type>(nb));
  int rc{ -1 };

  rc = modbus_read_registers(modbus_connection_, addr, nb, tab_reg.data());
  if (rc == -1)
  {
    std::ostringstream err_stream;
    err_stream << "Failed to read " << nb;
    err_stream << " registers starting from " << addr;
    err_stream << " with err: " << modbus_strerror(errno);
    ROS_ERROR_STREAM_NAMED("LibModbusClient", err_stream.str());
    throw ModbusExceptionDisconnect(err_stream.str());
  }

  return tab_reg;
}

RegCont LibModbusClient::writeReadHoldingRegister(const int write_addr, const RegCont& write_reg, const int read_addr,
                                                  const int read_nb)
{
  if (modbus_connection_ == nullptr)
  {
    throw ModbusExceptionDisconnect("Modbus disconnected!");
  }

  if (read_nb < 0)
  {
    throw std::invalid_argument("Argument \"read_nb\" must not be negative");
  }
  RegCont read_reg(static_cast<RegCont::size_type>(read_nb));

  if (write_reg.size() > std::numeric_limits<int>::max())
  {
    throw std::invalid_argument("Argument \"write_reg\" must not exceed max value of type \"int\"");
  }

  int rc{ -1 };
  rc = modbus_write_and_read_registers(modbus_connection_, write_addr, static_cast<int>(write_reg.size()),
                                       write_reg.data(), read_addr, read_nb, read_reg.data());
  ROS_DEBUG_NAMED("LibModbusClient", "modbus_write_and_read_registers: writing from %i %i registers\
                                      and reading from %i %i registers",
                  write_addr, static_cast<int>(write_reg.size()), read_addr, read_nb);
  if (rc == -1)
  {
    std::string err = "Failed to write and read modbus registers: ";
    err.append(modbus_strerror(errno));
    ROS_ERROR_STREAM_NAMED("LibModbusClient", err);
    throw ModbusExceptionDisconnect(err);
  }

  return read_reg;
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

bool checkIPConnection(const char* ip, const unsigned int port)
{
  long result;
  int sockfd;
  struct sockaddr_in serv_addr;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);  // Create socket for connection testing purpose
  bzero((char*)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons((short unsigned int)port);

  result = fcntl(sockfd, F_GETFL, NULL);
  result |= O_NONBLOCK;
  fcntl(sockfd, F_SETFL, result);  // set connection to non blocking, no timeout
  serv_addr.sin_addr.s_addr = inet_addr(ip);

  if (connect(sockfd, (const sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
  {
    return false;
  }
  ::close(sockfd);
  std::this_thread::sleep_for(std::chrono::duration<double>(1));  // wait one second to grant a free port

  return true;
}

}  // namespace prbt_hardware_support
