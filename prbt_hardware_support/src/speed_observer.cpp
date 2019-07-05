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

#include <prbt_hardware_support/speed_observer.h>
#include <tf/transform_listener.h>

namespace prbt_hardware_support
{

SpeedObserver::SpeedObserver(ros::NodeHandle& nh)
  : nh_(nh)
{

}

void SpeedObserver::startObserving(double frequency) const
{
  std::string tracking_frame = "prbt_tcp";
  std::string observation_frame = "prbt_link_1";


  ros::Rate r(frequency); // 10 hz
  tf::TransformListener listener;

  while (ros::ok())
  {
    ros::spinOnce();
    if(!listener.canTransform(tracking_frame, observation_frame, ros::Time(0)))
    {
      ROS_ERROR("Cannot transform!");
      continue;
    }

    try
    {
      geometry_msgs::Twist twist;
      listener.lookupTwist("prbt_tcp", "prbt_link_1", ros::Time(0), ros::Duration(0.1), twist);
      ROS_ERROR_STREAM(twist);
    }
    catch(const tf2::ExtrapolationException &ex)
    {
      ROS_ERROR_STREAM(ex.what());
    }



    r.sleep();
  }
}

}
