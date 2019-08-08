/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#include <ros/ros.h>
#include <prbt_hardware_support/speed_observer.h>

using namespace prbt_hardware_support;

static const std::string REFERENCE_FRAME_PARAM_NAME{"reference_frame"};
static const std::string REFERENCE_FRAME_PARAM_DEFAULT{"prbt_base_link"};

/**
 * @brief Read requested parameters, start and initialize the prbt_hardware_support::SpeedObserver
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "speed_observer");

  ros::NodeHandle nh {"~"};
  std::string reference_frame;
  nh.param<std::string>(REFERENCE_FRAME_PARAM_NAME, reference_frame, REFERENCE_FRAME_PARAM_DEFAULT);

  std::vector<std::string> frames_to_observe = std::vector<std::string>({"prbt_tcp"});

  prbt_hardware_support::SpeedObserver observer(
    nh,
    reference_frame,
    frames_to_observe
  );
  observer.startObserving(10);

  ros::spin();

  return EXIT_SUCCESS;

}
