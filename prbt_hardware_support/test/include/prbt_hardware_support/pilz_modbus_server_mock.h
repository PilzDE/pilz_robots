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

#ifndef PRBT_HARDWARE_SUPPORT_PILZ_MODBUS_SERVER_MOCK_H
#define PRBT_HARDWARE_SUPPORT_PILZ_MODBUS_SERVER_MOCK_H

#include <mutex>
#include <atomic>
#include <cstdint>
#include <vector>
#include <thread>
#include <condition_variable>

#include <modbus/modbus.h>

#include <ros/console.h>

namespace prbt_hardware_support
{

/**
 * @class PilzModbusServerMock offers a modbus server and read/write functionality
 * via subscription publication.
 */
class PilzModbusServerMock
{
public:
  PilzModbusServerMock(const unsigned int& holding_register_size);

  ~PilzModbusServerMock();

  /**
   * @brief Start the modbus server and make it accessible for clients
   *
   * @param ip The ip the server attaches to
   * @param port The used port
   */
  void start(const char* ip, const unsigned int port);

  /**
   * @brief Start the modbus server asynchronously and make it accessible for clients
   *
   * @param ip The ip the server attaches to
   * @param port The used port
   *
   * Returns as soon as it is running
   */
  void startAsync(const char* ip, const unsigned int port);

  void setHoldingRegister(std::initializer_list< std::pair<unsigned int, uint16_t> > reg_list);

  /**
   * @brief Set the values in the holding register
   *
   * @param data The values in the holding register
   * @param start_index The index from where the value is set, other values remain untouched.
   */
  void setHoldingRegister(const std::vector<uint16_t>& data, unsigned int start_index);

  /**
   * @brief Terminate the Server. Reading or connecting to it will fail.
   */
  void terminate();

  /**
   * @brief Allocates needed resources for running the server.
   *
   * This needs to be called before a successful run()
   *
   * @param ip The ip the server attaches to
   * @param port The used port
   * @return True on success, false otherwise
   */
  bool init(const char* ip, unsigned int port);

  /**
   * @brief Run the server and publish the register values as messages.
   * The value of the modbus register is read in a loop.
   * Once in a loop pass the value is published as a message.
   * In order for clients to differentiate between messages notifying about
   * changes in the register the timestamp of the message is only changed
   * if a register value changed.
   */
  void run();

private:
  const unsigned int holding_register_size_;

private:

  //! Modbus
  int socket_ {-1};
  modbus_t *modbus_connection_ {nullptr};
  modbus_mapping_t *mb_mapping_ {nullptr};

  std::atomic_bool terminate_ {false};

  std::mutex modbus_register_access_mutex;

  std::mutex running_mutex_;
  std::condition_variable running_cv_;

  std::thread thread_;

private:
  static constexpr uint32_t DISCONNECT_TIMEOUT_IN_SEC     {1};
  static constexpr uint32_t DISCONNECT_TIMEOUT_IN_USEC    {0};

  static constexpr uint32_t RESPONSE_TIMEOUT_IN_SEC       {0};
  static constexpr uint32_t RESPONSE_TIMEOUT_IN_USEC      {10000};

};

inline void PilzModbusServerMock::terminate()
{
  terminate_ = true;

  if(thread_.joinable())
  {
    thread_.join();
  }
}

}
#endif // PRBT_HARDWARE_SUPPORT_PILZ_MODBUS_SERVER_MOCK_H
