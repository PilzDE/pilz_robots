/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <trajectory_interface/quintic_spline_segment.h>
#include <pilz_control/pilz_joint_trajectory_controller.h>

namespace position_controllers
{

  /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to a \b position interface.
   */
  typedef pilz_joint_trajectory_controller::PilzJointTrajectoryController
                        <trajectory_interface::QuinticSplineSegment<double>, hardware_interface::PositionJointInterface>
          PilzJointTrajectoryController;

}  // namespace position_controllers

PLUGINLIB_EXPORT_CLASS(position_controllers::PilzJointTrajectoryController, controller_interface::ControllerBase)
