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

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>

static const std::string CONTROLLER_NS_PARAM_NAME {"controller_ns_string"};
static const std::string JOINT_NAME {"joint1"};

/**
 * @brief The RobotMock used by the integrationtest of the pilz_joint_trajectory_controller
 * Registers a single JointStateHandle with the interface to allow interaction with the controller_manager
 */
class RobotMock : public hardware_interface::RobotHW
{
public:
  RobotMock()
  {
    // register joint interface
    pos_ = new double();
    vel_ = new double();
    eff_ = new double();
    hardware_interface::JointStateHandle jnt_state_handle {JOINT_NAME, pos_, vel_, eff_};
    cmd_ = new double();
    hardware_interface::JointHandle jnt_handle {jnt_state_handle, cmd_};

    pos_jnt_interface.registerHandle(jnt_handle);

    registerInterface(&pos_jnt_interface);
  }

  void read()
  {
  }

  void write()
  {
    *vel_ = (*cmd_ - *pos_) / getPeriod().toSec();
    *pos_ = *cmd_;
  }

  ros::Time getTime() const
  {
    return ros::Time::now();
  }

  ros::Duration getPeriod() const
  {
    return ros::Duration(0.01);
  }

private:
  double* pos_;
  double* vel_;
  double* eff_;
  double* cmd_;
  hardware_interface::PositionJointInterface pos_jnt_interface;
};

// Runs as node
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_mock");

  std::string controller_ns;
  ros::param::get(CONTROLLER_NS_PARAM_NAME, controller_ns);
  ros::NodeHandle nh {controller_ns};

  RobotMock robot;
  controller_manager::ControllerManager cm(&robot, nh);


  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
    robot.read();
    cm.update(robot.getTime(), robot.getPeriod());
    robot.write();
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
