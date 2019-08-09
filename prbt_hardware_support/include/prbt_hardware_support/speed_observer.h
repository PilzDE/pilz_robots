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

#ifndef SPEED_OBSERVER_H
#define SPEED_OBSERVER_H

#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>

#include <prbt_hardware_support/FrameSpeeds.h>

namespace prbt_hardware_support
{

class SpeedObserver
{
public:
  SpeedObserver(
      ros::NodeHandle& nh,
      std::string& reference_frame,
      std::vector<std::string>& frames_to_observe
      );
  void startObserving(double frequency);

private:
  ros::NodeHandle nh_;
  ros::Publisher frame_speeds_pub;
  uint32_t seq{0};
  const std::string reference_frame_;
  const std::vector<std::string> frames_to_observe_;
  std::vector<geometry_msgs::Vector3> velocities;
  std::vector<double> speeds;

  FrameSpeeds makeFrameSpeedsMessage(std::vector<double>& speeds);
  static double speedFromVelocityVector(const geometry_msgs::Vector3& v);

};

} // namespace prbt_hardware_support
#endif // SPEED_OBSERVER_H
