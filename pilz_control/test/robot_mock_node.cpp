/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
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

#include <controller_manager/controller_manager.h>

#include "robot_mock.h"

static const std::string CONTROLLER_NS_PARAM_NAME{ "controller_ns_string" };
static constexpr double UPDATE_PERIOD{ 0.01 };

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_mock");

  std::string controller_ns;
  ros::param::get(CONTROLLER_NS_PARAM_NAME, controller_ns);
  ros::NodeHandle nh{ controller_ns };

  RobotMock robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / UPDATE_PERIOD);
  ros::Duration update_period{ UPDATE_PERIOD };
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
    robot.read();
    cm.update(ros::Time::now(), update_period);
    robot.write(update_period);
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
