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

#include <std_srvs/Empty.h>

#include <pilz_utils/wait_for_service.h>

static constexpr int WAITING_TIME_S{ 1 };

bool dummyServiceCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
  return true;
}

/**
 * @brief Tests that an asynchronous call to waitForService waits for the service to come up.
 */
TEST(WaitForServiceTests, testAsyncCall)
{
  using namespace pilz_utils;
  std::string service_name{ "/test_service" };

  auto wait_future = std::async([service_name]() {
    waitForService(service_name);
    return true;
  });

  EXPECT_EQ(std::future_status::timeout, wait_future.wait_for(std::chrono::seconds(WAITING_TIME_S)));

  ros::NodeHandle nh;
  ros::ServiceServer server = nh.advertiseService(service_name, dummyServiceCallback);

  EXPECT_TRUE(wait_future.get());

  server.shutdown();
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unittest_wait_for_service");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
