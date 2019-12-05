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

#ifndef PILZ_CARTESIAN_SPEED_CONTROLLER_H
#define PILZ_CARTESIAN_SPEED_CONTROLLER_H

#include <future>
#include <vector>

#include <ros/ros.h>

#include <joint_state_controller/joint_state_controller.h>
#include <pilz_control/cartesian_speed_monitor.h>

namespace pilz_cartesian_speed_observing_controller
{

typedef std::vector<double> JointPositions;

class PilzCartesianSpeedObservingController: public controller_interface::Controller<hardware_interface::JointStateInterface>
{
public:
  PilzCartesianSpeedObservingController() 
  {

  }

  virtual bool init(hardware_interface::JointStateInterface* hw,
                    ros::NodeHandle&                         root_nh,
                    ros::NodeHandle&                         controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:
  void triggerHold();

private:
  std::vector<hardware_interface::JointStateHandle> joint_state_;
  unsigned int num_hw_joints_;
  std::shared_ptr < pilz_control::CartesianSpeedMonitor > cartesian_speed_monitor;
  JointPositions last_positions;
  bool first_run {true};
  ros::ServiceClient hold_client_;
  std::future<bool> hold_success_;
  bool is_hold_running_{false};
};

}  // namespace pilz_cartesian_speed_observing_controller

#endif  // PILZ_CARTESIAN_SPEED_CONTROLLER_H
