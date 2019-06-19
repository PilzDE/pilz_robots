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

#ifndef LIBMODBUS_CLIENT_H
#define LIBMODBUS_CLIENT_H

#include <modbus/modbus.h>

#include <vector>

#include <prbt_hardware_support/modbus_client.h>

namespace prbt_hardware_support
{

/**
 * @brief Wrapper around libmodbus, see https://libmodbus.org/
 *
 * Used by PilzModbusReadClient to access a modbus server
 */
class LibModbusClient : public ModbusClient
{
public:

  virtual ~LibModbusClient();

  /**
   * @brief Initialize the modbus client by connecting to the server
   *
   * @param ip of the server
   * @param port to use
   * @return true if the connection to the server succeeded
   * @return false if the connection to the server failed
   */
  bool init(const char* ip, unsigned int port);

  /**
   * @brief Set the response timeout
   *
   * Interval to wait for a response from the server if within this timespan no answer is received readHoldingRegisters
   * will throw a ModbusExceptionDisconnect
   *
   * @param timeout_ms in ms
   */
  void setResponseTimeoutInMs(unsigned int timeout_ms);

  /**
   * @brief Get the response timeout
   *
   * @return response timeout
   */
  unsigned int getResponseTimeoutInMs();

  /**
   * @brief Read the holding registers
   *
   * @param addr starting address to read from
   * @param nb number of registers to read
   * @throw ModbusExceptionDisconnect if a disconnect from the server happens
   * @return std::vector<uint16_t> containing the register contents
   */
  std::vector<uint16_t> readHoldingRegister(int addr, int nb);

  /**
   * @brief Close connection with server
   */
  void close();

private:
  modbus_t *modbus_connection_ {nullptr};
};

}

#endif // LIBMODBUS_CLIENT_H
