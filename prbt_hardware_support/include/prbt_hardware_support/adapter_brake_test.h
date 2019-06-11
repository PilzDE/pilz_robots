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

#ifndef ADAPTER_BRAKE_TEST_H
#define ADAPTER_BRAKE_TEST_H

#include <ros/ros.h>
#include <prbt_hardware_support/IsBrakeTestRequired.h>

namespace prbt_hardware_support
{

/**
 * @brief Publishes a message informing about a required brake test.
 *
 * A message of type std_msgs::Bool is published which is True if a brake test is required and False otherwise.
 */
class AdapterBrakeTest
{
public:
  AdapterBrakeTest(ros::NodeHandle& nh);

protected:
	void init();
  virtual void updateBrakeTestRequiredState(bool brake_test_required);
  bool isBrakeTestRequired(IsBrakeTestRequired::Request&, IsBrakeTestRequired::Response& response);

private:
	//! Is the node initialized?
	bool initialized_;

	//! Store the current state of whether a brake test is required
	bool brake_test_required_;

	//! The node handle
	ros::NodeHandle& nh_;

	//! Server serving a service to ask whether a brake test is currently required
	ros::ServiceServer is_brake_test_required_server_;

};

} // namespace prbt_hardware_support
#endif // ADAPTER_BRAKE_TEST_H
