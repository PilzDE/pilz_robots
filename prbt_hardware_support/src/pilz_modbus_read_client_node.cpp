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

#include <stdlib.h>
#include <string>

#include <ros/ros.h>

#include <prbt_hardware_support/libmodbus_client.h>
#include <prbt_hardware_support/pilz_modbus_read_client.h>
#include <prbt_hardware_support/param_names.h>
#include <prbt_hardware_support/pilz_modbus_read_client_exception.h>

static constexpr int32_t MODBUS_CONNECTION_RETRIES_DEFAULT {10};
static constexpr double MODBUS_CONNECTION_RETRY_TIMEOUT_S_DEFAULT {1.0};

using namespace prbt_hardware_support;

/**
 * @brief Read requested parameters, start and initialize the prbt_hardware_support::PilzModbusReadClient
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "modbus_read_node");

  ros::NodeHandle nh {"~"};

  // LCOV_EXCL_START Simple parameter reading not analyzed
  std::string ip {};
  if ( !nh.getParam(PARAM_MODBUS_SERVER_IP_STR, ip) )
  {
    ROS_ERROR("No modbus server ip given. Abort.");
    return EXIT_FAILURE;
  }

  int port {0};
  if ( !nh.getParam(PARAM_MODBUS_SERVER_PORT_STR, port) )
  {
    ROS_ERROR("No modbus server port given. Abort.");
    return EXIT_FAILURE;
  }

  int num_registers_to_read {0};
  if ( !nh.getParam(PARAM_NUM_REGISTERS_TO_READ_STR, num_registers_to_read) )
  {
    ROS_ERROR("Numbers of registers to read not given. Abort.");
    return EXIT_FAILURE;
  }

  int index_of_first_register {0};
  if ( !nh.getParam(PARAM_INDEX_OF_FIRST_REGISTER_TO_READ_STR, index_of_first_register) )
  {
    ROS_ERROR("Index of first register not given. Abort.");
    return EXIT_FAILURE;
  }

  int32_t modbus_connection_retries {MODBUS_CONNECTION_RETRIES_DEFAULT};
  nh.param<int32_t>(PARAM_MODBUS_CONNECTION_RETRIES, modbus_connection_retries, MODBUS_CONNECTION_RETRIES_DEFAULT);

  double modbus_connection_retry_timeout_s {MODBUS_CONNECTION_RETRY_TIMEOUT_S_DEFAULT};
  nh.param<double>(PARAM_MODBUS_CONNECTION_RETRY_TIMEOUT, modbus_connection_retry_timeout_s,
           MODBUS_CONNECTION_RETRY_TIMEOUT_S_DEFAULT);

  // LCOV_EXCL_STOP

  prbt_hardware_support::PilzModbusReadClient modbus_client(nh,
                                                  static_cast<uint32_t>(num_registers_to_read),
                                                  static_cast<uint32_t>(index_of_first_register),
                                                  std::unique_ptr<LibModbusClient>(new LibModbusClient()));

  ROS_DEBUG_STREAM("Modbus client IP: " << ip << " | Port: " << port);
  ROS_DEBUG_STREAM("Number of registers to read: " << num_registers_to_read
                   << "| first register: " << index_of_first_register);


  bool res = modbus_client.init(ip.c_str(), static_cast<unsigned int>(port),
                                            static_cast<unsigned int>(modbus_connection_retries),
                                ros::Duration(modbus_connection_retry_timeout_s));

  ROS_DEBUG_STREAM("Connection with modbus server " << ip << ":" << port << " established");

  // LCOV_EXCL_START inside this main ignored, tested multiple times in the unittest
  if(!res)
  {
    ROS_ERROR_STREAM("Connection to modbus server " << port << ":" << ip << " could not be established");
    return EXIT_FAILURE;
  }

  try
  {
    modbus_client.run();
  }
  catch(PilzModbusReadClientException& e)
  {
    ROS_ERROR_STREAM(e.what());
    return EXIT_FAILURE;
  }
  // LCOV_EXCL_STOP

  // If the client stop we don't want to kill this node since
  // with required=true this would kill all nodes and no STOP1 would be performed in case of a disconnect.
  ros::spin();

  return EXIT_SUCCESS;

}
