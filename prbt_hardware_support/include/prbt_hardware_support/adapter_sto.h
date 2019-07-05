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

#ifndef ADAPTER_STO_H
#define ADAPTER_STO_H

#include <memory>
#include <string>

#include <boost/optional.hpp>

#include <ros/ros.h>

#include <prbt_hardware_support/modbus_api_spec.h>

namespace prbt_hardware_support
{

/**
 * @brief Stores the last reported STO stated and reacts to changes of the STO.
 * Reactions:
 *  - STO == false:   Perfrom Stop 1
 *  - STO == true:    unhold Controller + recover Drives
 *
 * @note PilzStoModbusAdapterNode::DURATION_BETWEEN_HOLD_AND_DISABLE_MS
 * specifies the time between holding the controller and halting the driver.
 * This allows the controller to perform a smooth stop before a driver disable
 * enables the breaks.
 */
class AdapterSto
{
public:
  AdapterSto(ros::NodeHandle& nh);
  virtual ~AdapterSto();

public:
  static const std::string HOLD_SERVICE;
  static const std::string UNHOLD_SERVICE;
  static const std::string RECOVER_SERVICE;
  static const std::string HALT_SERVICE;

protected:
  /**
   * @brief Stops the controller and halts the driver after.
   */
  void performStop();

  /**
   * @brief Update stored STO value and react to STO changes.
   * Reactions:
   *  - STO == false:   Perform Stop 1
   *  - STO == true:    unhold Controller + recover Drives
   */
  void updateSto(const bool sto);

private:
  /**
   * @brief Calls the unhold service of the controller.
   */
  void unholdController();

  /**
   * @brief Calls the recover service of the drives.
   */
  void recoverDrives();

private:
  boost::optional<bool> sto_ {boost::none};

  //! ServiceClient attached to the controller <code>/hold</code> service
  ros::ServiceClient hold_srv_client_;

  //! ServiceClient attached to the controller <code>/unhold</code> service
  ros::ServiceClient unhold_srv_client_;

  //! ServiceClient attached to the driver <code>/recover</code> service
  ros::ServiceClient recover_srv_client_;

  //! ServiceClient attached to the driver <code>/halt</code> service
  ros::ServiceClient halt_srv_client_;

private:
  /**
   * @brief Specifies the time between holding the controller and
   * disabling the driver. This allows the controller to perform a smooth stop
   * before a driver disable enables the breaks.
   */
  static constexpr int DURATION_BETWEEN_HOLD_AND_DISABLE_MS {200};

  static constexpr double WAIT_FOR_SERVICE_TIMEOUT_S {5.0};
};

}
#endif // ADAPTER_STO_H
