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
#include <future>
#include <string>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <prbt_hardware_support/service_client_factory.h>

bool dummyServiceCallback(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  return true;
}

/**
 * @brief Tests that an asynchronous call to ServiceClientFactory::create waits for
 *        the service to come up and returns a service client.
 */
TEST(ServiceClientFactoryIntegrationTest, testCreate)
{
  using namespace prbt_hardware_support;
  std::string service_name{"/test_service"};

  auto create_future = std::async(ServiceClientFactory::create<std_srvs::Empty>, service_name, false);

  ros::NodeHandle nh;
  ros::ServiceServer server = nh.advertiseService(service_name, dummyServiceCallback);

  create_future.wait();

  ros::ServiceClient client {create_future.get()};
  EXPECT_TRUE(client.exists());
  EXPECT_EQ(service_name, client.getService());

  client.shutdown();
  server.shutdown();
}

/**
 * @brief Tests that a persistent service client can be created via ServiceClientFactory::create.
 *
 * Test Sequence:
 *  1. Call ServiceClientFactory::create with persistent=true without service being advertised
 *  2. Advertise service and create persistent client
 *
 * Expected Results:
 *  1. Service client handle is not valid.
 *  2. Service client handle is valid and the connection is persistent.
 */
TEST(ServiceClientFactoryIntegrationTest, testCreatePersistent)
{
  using namespace prbt_hardware_support;
  std::string service_name{"/test_service"};

  ros::ServiceClient client = ServiceClientFactory::create<std_srvs::Empty>(service_name, true);
  EXPECT_FALSE(client.isValid());

  ros::NodeHandle nh;
  ros::ServiceServer server = nh.advertiseService(service_name, dummyServiceCallback);

  client = ServiceClientFactory::create<std_srvs::Empty>(service_name, true);

  EXPECT_TRUE(client.isValid());
  EXPECT_TRUE(client.isPersistent());
  EXPECT_EQ(service_name, client.getService());

  client.shutdown();
  server.shutdown();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "integrationtest_stop1");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
