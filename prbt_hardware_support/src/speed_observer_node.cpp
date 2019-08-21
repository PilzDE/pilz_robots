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
#include <urdf/model.h>

using namespace prbt_hardware_support;

static const std::string REFERENCE_FRAME_PARAM_NAME{"reference_frame"};
static const std::string REFERENCE_FRAME_PARAM_DEFAULT{"prbt_base_link"};
static const std::string ADDITIONAL_FRAMES_PARAM_NAME{"additional_frames"};
static const std::string ROBOT_DESCRIPTION_PARAM_NAME{"robot_description"};
static const double OBSERVATION_FREQUENCY{10};

/**
 * @brief Read requested parameters, start and initialize the prbt_hardware_support::SpeedObserver
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "speed_observer");
  ros::NodeHandle pnh {"~"};
  ros::NodeHandle nh;

  urdf::Model m;
  m.initParam(ROBOT_DESCRIPTION_PARAM_NAME);
  ROS_DEBUG_STREAM("reference_frame: " << m.getRoot()->name);
  std::string reference_frame =  m.getRoot()->name;
  std::vector<urdf::LinkSharedPtr > links;
  std::vector<std::string> frames_to_observe;
  m.getLinks(links);
  ROS_DEBUG_STREAM("frames_to_observe (from urdf):");
  for( const auto &l : links){
    if(l->name != reference_frame)  // no need to monitor the ref frame
    {
      ROS_DEBUG_STREAM(" - " << l->name);
      frames_to_observe.push_back(l->name);
    }
  }
  std::vector<std::string> additional_frames;
  pnh.getParam(ADDITIONAL_FRAMES_PARAM_NAME, additional_frames);
  ROS_DEBUG_STREAM("frames_to_observe (additional_frames): ");
  for( const auto &f : additional_frames){
    ROS_DEBUG_STREAM(" - " << f);
  }
  frames_to_observe.insert(frames_to_observe.end(), additional_frames.begin(), additional_frames.end());

  prbt_hardware_support::SpeedObserver observer(
    nh,
    reference_frame,
    frames_to_observe
  );
  observer.startObserving(OBSERVATION_FREQUENCY);

  return EXIT_SUCCESS;

}
