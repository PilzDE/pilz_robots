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

#ifndef BRAKE_TEST_ANNOUNCER_H
#define BRAKE_TEST_ANNOUNCER_H

#include <ros/ros.h>

namespace prbt_hardware_support
{

/**
 * @brief Publishes a message informing about a required brake test.
 *
 * A message of type std_msgs::Bool is published which is True if a brake test is required and False otherwise.
 */
class BrakeTestAnnouncer
{
public:
  BrakeTestAnnouncer(ros::NodeHandle& nh);

protected:
  virtual void sendBrakeTestRequiredMsg(bool brake_test_required) const;

private:
  //! Publisher used to inform potential listeners about the current
  //! status of the brake test requirement.
  ros::Publisher brake_test_required_pub_;

};

} // namespace prbt_hardware_support
#endif // BRAKE_TEST_ANNOUNCER_H
