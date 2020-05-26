/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PILZ_UTILS_WAIT_FOR_MESSAGE_H
#define PILZ_UTILS_WAIT_FOR_MESSAGE_H

#include <string>

#include <ros/ros.h>
#include <ros/duration.h>

#include <pilz_utils/wait_for_timeouts.h>

namespace pilz_utils
{
/**
 * @brief Waits (infinitely) until a message arrives on the specified topic.
 */
template <class T>
static void waitForMessage(const std::string topic_name, const double retry_timeout = DEFAULT_RETRY_TIMEOUT,
                           const double msg_output_period = DEFAULT_MSG_OUTPUT_PERIOD)
{
  while ((ros::topic::waitForMessage<T>(topic_name, ros::Duration(retry_timeout)) == nullptr) && ros::ok())
  {
    ROS_DEBUG_STREAM_DELAYED_THROTTLE(msg_output_period, "Waiting for message on topic \"" + topic_name + "\"...");
  }
}

}  // namespace pilz_utils

#endif  // PILZ_UTILS_WAIT_FOR_MESSAGE_H
