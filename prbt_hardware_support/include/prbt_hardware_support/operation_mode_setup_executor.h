/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
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

#ifndef OPERATION_MODE_SETUP_EXECUTOR_H
#define OPERATION_MODE_SETUP_EXECUTOR_H

#include <atomic>

#include <ros/ros.h>

#include <pilz_msgs/GetSpeedOverride.h>

#include <prbt_hardware_support/OperationModes.h>
#include <prbt_hardware_support/set_speed_limit_func_decl.h>
#include <prbt_hardware_support/get_operation_mode_func_decl.h>


namespace prbt_hardware_support
{

/**
 * @brief Sets the allowed speed limit for each frame based on
 * the current operation mode.
 */
class OperationModeSetupExecutor
{
public:
  /**
   * @brief Ctor.
   *
   * @param speed_limit_t1 limit Max allowed speed for each frame in operation mode T1.
   *
   * @param speed_limit_auto limit Max allowed speed for each frame in operation mode AUTOMATIC.
   *
   * @param set_speed_limit_func Function allowing to report the speed limit
   * change to the system
   */
  OperationModeSetupExecutor(const double& speed_limit_t1,
                             const double& speed_limit_auto,
                             const SetSpeedLimitFunc& set_speed_limit_func);

public:
  /**
   * @brief Function to be called whenever a new operation mode is set.
   */
  void updateOperationMode(const OperationModes& operation_mode);

  bool getSpeedOverride(pilz_msgs::GetSpeedOverride::Request& /*req*/,
                        pilz_msgs::GetSpeedOverride::Response& response);

private:
  //! The allowed speed limit in operation mode T1.
  const double speed_limit_t1_;
  //! The allowed speed limit in operation mode AUTOMATIC.
  const double speed_limit_auto_;

  //! The active speed override
  std::atomic<double> speed_override_{0.0};

  //! Function used to propagate speed limit changes into the system.
  SetSpeedLimitFunc set_speed_limit_func_;
  //! Time stamp of the last received operation mode.
  ros::Time time_stamp_last_op_mode_ {ros::Time(0)};
};


} // namespace prbt_hardware_support

#endif // OPERATION_MODE_SETUP_EXECUTOR_H
