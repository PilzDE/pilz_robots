/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
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

#ifndef PILZ_UTILS_SLEEP
#define PILZ_UTILS_SLEEP

#include <ros/ros.h>
#include <ros/time.h>

namespace pilz_utils
{
/**
 * \brief Replace the sim-time part of ros::Duration::sleep() by a progressing in time.
 *
 * Use this in production code in order to facilitate tests with simulated time.
 *
 * When using simulated time, ros::Duration::sleep() has to be synchronized with calls to ros::Time::setNow(),
 * else it will block. This sleep function simply performs the progressing in time itself, such that it never blocks.
 * \note Simulated time is valid, if it is non-zero. Otherwise no sleep is performed.
 * \note Modifying the simulated time in parallel to this function will lead to unpredictable results.
 * \return Returns true if sleep was performed and false if not.
 */
static bool sleep(const ros::Duration& sleep_duration)
{
  if (ros::Time::isSystemTime())
  {
    return sleep_duration.sleep();
  }

  if (ros::Time::isValid())
  {
    ros::Time::setNow(ros::Time::now() + sleep_duration);
    return true;
  }

  return false;
}

}  // namespace pilz_utils

#endif
