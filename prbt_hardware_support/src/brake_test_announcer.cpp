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

#include <prbt_hardware_support/brake_test_announcer.h>
#include <std_msgs/Bool.h>


namespace prbt_hardware_support
{

static const std::string SERVICE_NAME_IS_BRAKE_TEST_REQUIRED = "/prbt/is_brake_test_required";

BrakeTestAnnouncer::BrakeTestAnnouncer(ros::NodeHandle& nh)
  : initialized_(false)
  , nh_(nh) {
}

void BrakeTestAnnouncer::init() {
	is_brake_test_required_server_ = nh_.advertiseService(SERVICE_NAME_IS_BRAKE_TEST_REQUIRED,
	                                                      &BrakeTestAnnouncer::isBrakeTestRequired,
	                                                      this);
}

void BrakeTestAnnouncer::updateBrakeTestRequiredState(bool brake_test_required) {
	// when the first data is received, the node is initialized (i.e. the service advertised)
	if(!initialized_) {
		init();
		initialized_ = true;
	}
	brake_test_required_ = brake_test_required;

  if(brake_test_required_){
    ROS_INFO("Brake Test required.");
  }
  else {
    ROS_INFO("Brake Test not required.");
  }
}

bool BrakeTestAnnouncer::isBrakeTestRequired(IsBrakeTestRequired::Request& req,
				                                     IsBrakeTestRequired::Response& res) {
	res.result = brake_test_required_;
	return true;
}

}
