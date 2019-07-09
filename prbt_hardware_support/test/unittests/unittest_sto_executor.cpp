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

#include <functional>

#include <ros/ros.h>

#include <prbt_hardware_support/sto_executor.h>
#include <prbt_hardware_support/service_client_mock.h>

TEST(STOExecutorTest, testDummy)
{
  using namespace prbt_hardware_support_tests;

  // for (limited) ros::Time functionality, no ROS communication
  ros::Time::init();

  typedef ServiceClientMockFactory<std_srvs::Trigger> MockFactory;
  typedef prbt_hardware_support::STOExecutorTemplated<ServiceClientMock<std_srvs::Trigger>> STOExecutor;

  MockFactory factory;

  STOExecutor sto_executor{std::bind(&MockFactory::create, &factory, std::placeholders::_1)};
}

int main(int argc, char **argv){
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}