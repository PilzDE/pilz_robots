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

#include <chrono>
#include <future>
#include <string>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <std_msgs/Empty.h>

#include <pilz_utils/wait_for_message.h>

static constexpr int WAITING_TIME_S{ 1 };
static constexpr int DEFAULT_QUEUE_SIZE{ 1 };

/**
 * @brief Tests that an asynchronous call to waitForMessage waits until someone publishes on the topic.
 */
TEST(WaitForMessageTests, testAsyncCall)
{
  using namespace pilz_utils;
  std::string topic_name{ "/test_topic" };

  auto wait_future = std::async([topic_name]() {
    waitForMessage<std_msgs::Empty>(topic_name);
    return true;
  });

  EXPECT_EQ(std::future_status::timeout, wait_future.wait_for(std::chrono::seconds(WAITING_TIME_S)));

  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<std_msgs::Empty>(topic_name, DEFAULT_QUEUE_SIZE);

  // publish message
  std_msgs::Empty msg;
  publisher.publish(msg);

  EXPECT_TRUE(wait_future.get());

  publisher.shutdown();
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unittest_wait_for_message");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
