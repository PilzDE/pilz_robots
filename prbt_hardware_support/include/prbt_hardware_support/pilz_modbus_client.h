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

#ifndef PRBT_HARDWARE_SUPPORT_CLIENT_NODE_H
#define PRBT_HARDWARE_SUPPORT_CLIENT_NODE_H

#include <atomic>
#include <mutex>

#include <boost/optional.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>

#include <prbt_hardware_support/modbus_client.h>
#include <prbt_hardware_support/register_container.h>

namespace prbt_hardware_support
{

/**
 * @brief Connects to a modbus server and publishes the received data into ROS.
 */
class PilzModbusClient
{
  typedef std::unique_ptr<ModbusClient> ModbusClientUniquePtr;

public:
  /**
   * @brief Sets up publisher. To open the modbus connection call PilzModbusClient::init.
   * @param nh Node handle.
   * @param num_registers_to_read Size of the data array.
   * @param index_of_first_register Offset of the modbus data.
   * @param modbus_client ModbusClient to use
   * @param response_timeout_ms Time to wait for a response from Modbus server.
   * @param modbus_read_topic_name Name under which modbus read message are published.
   * @param modbus_write_topic_name Name under which modbus write messages are received.
   * @param read_frequency_hz Defines how often Modbus registers are read in.
   */
  PilzModbusClient(ros::NodeHandle& nh,
                       const unsigned int num_registers_to_read,
                       const unsigned int index_of_first_register,
                       ModbusClientUniquePtr modbus_client,
                       unsigned int response_timeout_ms,
                       const std::string& modbus_read_topic_name,
                       const std::string& modbus_write_topic_name,
                       double read_frequency_hz = DEFAULT_MODBUS_READ_FREQUENCY_HZ);
public:
  /**
   * @brief Tries to connect to a modbus server.
   * @param ip
   * @param port
   * @return True if a connection is established, false otherwise.
   */
  bool init(const char* ip, unsigned int port);

  /**
   * @brief Tries to connect to a modbus server.
   * @param ip
   * @param port
   * @param retries Number of retries getting a connection to the server
   * @param timeout_ms between retries
   * @return True if a connection is established, false otherwise.
   */
  bool init(const char* ip, unsigned int port, unsigned int retries, ros::Duration timeout_ms);

  /**
   * @brief Publishes the register values as messages.
   *
   * The value of the modbus register is read in a loop.
   * Once a loop the value is published as a message.
   * In order for clients to differentiate between messages notifying about
   * changes in the register the timestamp of the message is only changed
   * if a register value changed.
   */
  void run();

  /**
   * @brief Ends the infinite loop started in method 'run()'.
   */
  void terminate();

  /**
   * @brief True if 'run()' method is active, false if 'run()' method is not active or,
   * currently, finishing.
   */
  bool isRunning();

private:
  void sendDisconnectMsg();
  void modbus_write_callback(std_msgs::UInt16MultiArray msg);

private:

  /**
   * @brief States of the Modbus-client.
   */
  enum State
  {
    not_initialized,
    initializing,
    initialized,
    running,
  };

  //! Number of registers which have to be read.
  const uint32_t NUM_REGISTERS_TO_READ;
  //! Index from which the registers have to be read.
  const uint32_t INDEX_OF_FIRST_REGISTER;

  //! Defines how long we wait for a response from the Modbus-server.
  const unsigned int RESPONSE_TIMEOUT_MS;
  //! Defines how often the Modbus registers are read in.
  const double READ_FREQUENCY_HZ;

private:
  static constexpr double DEFAULT_MODBUS_READ_FREQUENCY_HZ {500};
  static constexpr int DEFAULT_QUEUE_SIZE_MODBUS {1};

private:
  std::atomic<State> state_ {State::not_initialized};
  std::atomic_bool stop_run_ {false};
  ModbusClientUniquePtr modbus_client_;
  ros::Publisher modbus_pub_;
  ros::Subscriber modbus_sub_;

  std::mutex write_reg_msg_mutex_;
  boost::optional<std_msgs::UInt16MultiArray> write_reg_msg_ {boost::none};
  RegCont write_reg_;
};

inline void PilzModbusClient::terminate()
{
  stop_run_ = true;
}

inline bool PilzModbusClient::isRunning()
{
  return state_.load() == State::running;
}

inline void PilzModbusClient::modbus_write_callback(std_msgs::UInt16MultiArray msg)
{
  std::lock_guard<std::mutex> lock(write_reg_msg_mutex_);
  write_reg_msg_ = msg;
}

} // namespace prbt_hardware_support

#endif // PRBT_HARDWARE_SUPPORT_CLIENT_NODE_H
