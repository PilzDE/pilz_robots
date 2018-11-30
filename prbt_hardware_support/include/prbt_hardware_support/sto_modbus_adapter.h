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

#ifndef PILZ_STO_MODBUS_ADAPTER_NODE_H
#define PILZ_STO_MODBUS_ADAPTER_NODE_H

#include <memory>
#include <cstdint>
#include <string>

#include <ros/ros.h>

#include <message_filters/subscriber.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/update_filter.h>

#include <prbt_hardware_support/sto_modbus_adapter_exception.h>
#include <prbt_hardware_support/modbus_msg_sto_wrapper.h>

namespace prbt_hardware_support
{

/**
 * @brief Interprets modbus messages for STO handling.
 *
 * Subscribes to the /modbus_read topic. If a STO is received or a disconnect from the modbus server happend
 * it will hold the controller and halt the driver.
 *
 * @note PilzStoModbusAdapterNode::DURATION_BETWEEN_HOLD_AND_DISABLE_MS specifies the time between holding the
 * controller and halting the driver. This allows the controller to perform a smooth stop before a driver disable
 * enables the breaks.
 */
class PilzStoModbusAdapterNode
{
public:
  /**
   * @brief Constructor.
   * @param nh The node handle.
   * @param index_of_first_register_to_read Offset of the data in the modbus registers.
   */
  PilzStoModbusAdapterNode(ros::NodeHandle& nh);
  ~PilzStoModbusAdapterNode();

  static const std::string HOLD_SERVICE;
  static const std::string UNHOLD_SERVICE;
  static const std::string RECOVER_SERVICE;
  static const std::string HALT_SERVICE;

private:
  /**
   * @brief Extracts from Modbus message if STO is set or not and publishes information on the /stop1 topic.
   * @param modbus_msg
   */
  void modbusInMsgCallback(const ModbusMsgInStampedConstPtr& msg_raw);

  /**
   * @brief Stop the controller and halt the driver after PilzStoModbusAdapterNode::DURATION_BETWEEN_HOLD_AND_DISABLE_MS
   */
  void performStop();

private:

  /**
   * @brief Extracts from Modbus message if STO is set or not and publishes information on the /stop1 topic.
   * @param modbus_msg
   */
  void internalMsgCallback(const ModbusMsgStoWrapper& msg);

  const uint32_t NUM_REGISTERS_TO_READ {2};

  static constexpr unsigned int MODBUS_API_VERSION_REQUIRED {2};

  static constexpr int WAIT_FOR_SERVICE_TIMEOUT_S {5};

  /**
   * @brief Specifies the time between holding the controller and disabling the driver. This allows the controller
   * to perform a smooth stop before a driver disable enables the breaks.
   */
  const int DURATION_BETWEEN_HOLD_AND_DISABLE_MS {200};

  //! The node handle
  ros::NodeHandle nh_;

  /**
   * @brief Receives messages from PilzStoModbusAdapterNode::modbus_sub_ and filters consecutive messages with the
   * same timestamp.
   *
   * Passed messages are redirected to PilzStoModbusAdapterNode::modbusInMsgCallback.
   */
  std::shared_ptr< message_filters::UpdateFilter<ModbusMsgInStamped> > update_filter_;

  /**
   * @brief Subscribes to the modbus messages
   *
   * Subscribes to prbt_hardware_support::TOPIC_MODBUS_READ and redirects received messages
   * to PilzStoModbusAdapterNode::update_filter_ for filtering.
   */
  std::shared_ptr< message_filters::Subscriber<ModbusMsgInStamped> > modbus_sub_;

  //! ServiceClient attached to the controller <code>/hold</code> service
  ros::ServiceClient hold_srv_client_;

  //! ServiceClient attached to the controller <code>/unhold</code> service
  ros::ServiceClient unhold_srv_client_;

  //! ServiceClient attached to the driver <code>/recover</code> service
  ros::ServiceClient recover_srv_client_;

  //! ServiceClient attached to the driver <code>/halt</code> service
  ros::ServiceClient halt_srv_client_;

};

}
#endif // PILZ_STO_MODBUS_ADAPTER_NODE_H
