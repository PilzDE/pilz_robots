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

#include <prbt_hardware_support/operation_mode_setup_executor.h>

namespace prbt_hardware_support
{


OperationModeSetupExecutor::OperationModeSetupExecutor(const double& speed_limit_t1,
                                                       const double& speed_limit_auto,
                                                       const SetSpeedLimitFunc& set_speed_limit_func,
                                                       const GetOpModeFunc& get_op_mode_func)
  : speed_limit_t1_(speed_limit_t1)
  , speed_limit_auto_(speed_limit_auto)
  , set_speed_limit_func_(set_speed_limit_func)
{
  // Get initial operation mode because a call to the operation mode update
  // function only reports changes to the operation mode
  // (which might not happen for a long time).
  if (get_op_mode_func)
  {
    OperationModes om = get_op_mode_func();
    ROS_DEBUG("Operation Mode at init: %d", om.value);
    updateOperationMode(om);
  }
}


void OperationModeSetupExecutor::updateOperationMode(const OperationModes& operation_mode)
{
  ROS_DEBUG("updateOperationMode: %d", operation_mode.value);
  if (operation_mode.time_stamp <= time_stamp_last_op_mode_)
  {
    return;
  }
  time_stamp_last_op_mode_ = operation_mode.time_stamp;

  double speed_limit {0};
  switch(operation_mode.value)
  {
  case OperationModes::T1:
    speed_limit = speed_limit_t1_;
    break;
  case OperationModes::AUTO:
    speed_limit = speed_limit_auto_;
    break;
  default:
    break;
  }

  if (set_speed_limit_func_)
  {
    set_speed_limit_func_(speed_limit);
  }
}


} // namespace prbt_hardware_support
