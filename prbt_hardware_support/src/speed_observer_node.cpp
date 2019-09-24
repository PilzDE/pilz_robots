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
#include <urdf/model.h>

#include <prbt_hardware_support/wait_for_topic.h>
#include <prbt_hardware_support/speed_observer.h>

using namespace prbt_hardware_support;

static const std::string ADDITIONAL_FRAMES_PARAM_NAME{ "additional_frames" };
static const std::string ROBOT_DESCRIPTION_PARAM_NAME{ "robot_description" };
static const std::string SET_SPEED_LIMIT_SERVICE{ "set_speed_limit" };
static const double OBSERVATION_FREQUENCY{ 10 };

bool hasOnlyFixedParentJoints(const urdf::LinkSharedPtr &link)
{
  auto parent_link {link};
  while ( (parent_link->parent_joint != nullptr) && (parent_link->parent_joint->type == urdf::Joint::FIXED) )
  {
    parent_link = parent_link->getParent();
  }
  return parent_link->parent_joint == nullptr;
}

/**
 * @brief Read requested parameters, start and initialize the
 * prbt_hardware_support::SpeedObserver
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "speed_observer");
  ros::NodeHandle pnh{ "~" };
  ros::NodeHandle nh;

  urdf::Model model;
  model.initParam(ROBOT_DESCRIPTION_PARAM_NAME);
  ROS_DEBUG_STREAM("reference_frame: " << model.getRoot()->name);
  std::string reference_frame = model.getRoot()->name;
  std::vector<urdf::LinkSharedPtr> links;
  std::vector<std::string> frames_to_observe;
  model.getLinks(links);
  ROS_DEBUG_STREAM("Received the following frames to observer from urdf:");
  for (const auto& link : links)
  {
    if (!hasOnlyFixedParentJoints(link))
    {
      ROS_DEBUG_STREAM(" - " << link->name);
      frames_to_observe.push_back(link->name);
    }
  }
  std::vector<std::string> additional_frames;
  pnh.getParam(ADDITIONAL_FRAMES_PARAM_NAME, additional_frames);
  ROS_DEBUG_STREAM("Additional frames defined by user:");
  for (const auto& frame : additional_frames)
  {
    ROS_DEBUG_STREAM(" - " << frame);
  }
  frames_to_observe.insert(frames_to_observe.end(), additional_frames.begin(), additional_frames.end());

  SpeedObserver observer(nh, reference_frame, frames_to_observe);
  ros::ServiceServer set_speed_limit_server =
      nh.advertiseService(SET_SPEED_LIMIT_SERVICE, &SpeedObserver::setSpeedLimitCb, &observer);
  observer.startObserving(OBSERVATION_FREQUENCY);

  return EXIT_SUCCESS;
}
