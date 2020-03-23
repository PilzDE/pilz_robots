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

#include <array>
#include <sstream>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>

static const std::string CONTROLLER_NS_PARAM_NAME {"controller_ns_string"};

constexpr unsigned int NUM_JOINTS {2};
constexpr std::array<const char*, NUM_JOINTS> JOINT_NAMES = { "shoulder_to_right_arm", "shoulder_to_left_arm" };

struct JointData
{
  double pos {0.0};
  double vel {0.0};
  double eff {0.0};
  double cmd {0.0};
};

/**
 * @brief The RobotMock used by the integrationtest of the pilz_joint_trajectory_controller
 * Registers a single JointStateHandle with the interface to allow interaction with the controller_manager
 */
class RobotMock : public hardware_interface::RobotHW
{
public:
  RobotMock()
  {
    for (unsigned int i = 0; i<NUM_JOINTS; ++i)
    {
      std::ostringstream os;
      os << JOINT_NAMES.at(i);
      hardware_interface::JointStateHandle jnt_state_handle {os.str(),
            &(data_.at(i).pos), &(data_.at(i).vel), &(data_.at(i).eff)};
      hardware_interface::JointHandle jnt_handle {jnt_state_handle, &(data_.at(i).cmd)};
      pos_jnt_interface_.registerHandle(jnt_handle);
    }

    registerInterface(&pos_jnt_interface_);
  }

  void read()
  {
  }

  void write()
  {
    for (unsigned int i = 0; i<NUM_JOINTS; ++i)
    {
      data_.at(i).vel = (data_.at(i).cmd - data_.at(i).pos) / getPeriod().toSec();
      data_.at(i).pos = data_.at(i).cmd;
    }
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
  std::array<JointData, NUM_JOINTS> data_ { JointData(), JointData() };
  hardware_interface::PositionJointInterface pos_jnt_interface_;
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
