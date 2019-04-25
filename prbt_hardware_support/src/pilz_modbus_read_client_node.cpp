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
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <XmlRpcValue.h>

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

  int32_t modbus_connection_retries {MODBUS_CONNECTION_RETRIES_DEFAULT};
  nh.param<int32_t>(PARAM_MODBUS_CONNECTION_RETRIES, modbus_connection_retries, MODBUS_CONNECTION_RETRIES_DEFAULT);

  double modbus_connection_retry_timeout_s {MODBUS_CONNECTION_RETRY_TIMEOUT_S_DEFAULT};
  nh.param<double>(PARAM_MODBUS_CONNECTION_RETRY_TIMEOUT, modbus_connection_retry_timeout_s,
           MODBUS_CONNECTION_RETRY_TIMEOUT_S_DEFAULT);

  XmlRpc::XmlRpcValue rpc;
  if ( !nh.getParam(PARAM_CONFIG_NAMESPACE_STR, rpc) )
  {
    ROS_ERROR("No topic config given. Abort.");
    return EXIT_FAILURE;
  }


  ROS_DEBUG_STREAM("Modbus client IP: " << ip << " | Port: " << port);



  ROS_DEBUG_STREAM("Config: " << rpc.toXml().c_str());

  boost::thread_group tgroup;
  auto rpci = rpc.begin();
  for(; rpci != rpc.end(); ++rpci) {
      ROS_INFO("\n%s:\n----", rpci->first.c_str());

      int index_of_first_register_to_read = rpci->second[SUB_PARAM_INDEX_OF_FIRST_REGISTER_TO_READ_STR];
      ROS_INFO("%s: %d", SUB_PARAM_INDEX_OF_FIRST_REGISTER_TO_READ_STR.c_str(), index_of_first_register_to_read);

      int num_registers_to_read = rpci->second[SUB_PARAM_NUM_REGISTERS_TO_READ_STR];
      ROS_INFO("%s: %d", SUB_PARAM_NUM_REGISTERS_TO_READ_STR.c_str(), num_registers_to_read);

      int rate_hz = rpci->second[SUB_PARAM_RATE_HZ_STR];
      ROS_INFO("%s: %d", SUB_PARAM_RATE_HZ_STR.c_str(), rate_hz);

      PilzModbusReadClient modbus_client(nh,
                                         rpci->first,
                                         index_of_first_register_to_read,
                                         num_registers_to_read,
                                         rate_hz,
                                         std::unique_ptr<LibModbusClient>(new LibModbusClient()));

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
        tgroup.create_thread(
            boost::bind(&PilzModbusReadClient::run,
                        boost::ref(modbus_client))
        );
      }
      catch(PilzModbusReadClientException& e)
      {
        ROS_ERROR_STREAM(e.what());
        return EXIT_FAILURE;
      }
  }
  // LCOV_EXCL_STOP

  // If the client stop we don't want to kill this node since
  // with required=true this would kill all nodes and no STOP1 would be performed in case of a disconnect.

  tgroup.join_all();
  ros::spin();

  return EXIT_SUCCESS;

}
