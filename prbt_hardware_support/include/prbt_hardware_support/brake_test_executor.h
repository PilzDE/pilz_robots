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

#ifndef PRBT_HARDWARE_SUPPORT_BRAKE_TEST_EXECUTOR_H
#define PRBT_HARDWARE_SUPPORT_BRAKE_TEST_EXECUTOR_H

#include <ros/ros.h>

#include <prbt_hardware_support/BrakeTest.h>

namespace prbt_hardware_support
{

/**
 * @brief Triggers execution of brake tests only if the controller is not executing a trajectory.
 *
 */
class BrakeTestExecutor
{
public:
  BrakeTestExecutor(ros::NodeHandle& nh);

private:
  bool executeBrakeTest(BrakeTest::Request&, BrakeTest::Response& response);

private:
  ros::NodeHandle nh_;

  //! Service which can be called by the user to execute brake tests for all joints.
  ros::ServiceServer brake_test_srv_;

  ros::ServiceClient trigger_braketest_client_;

};

} // namespace prbt_hardware_support
#endif // PRBT_HARDWARE_SUPPORT_BRAKE_TEST_EXECUTOR_H
