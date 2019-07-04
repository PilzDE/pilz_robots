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
#include <map>
#include <string>

#include <gmock/gmock.h>

#include <ros/ros.h>

#include <prbt_hardware_support/sto_executor.h>

/**
 * @brief ...
 */
template <typename S>
class ServiceClientMock
{
public:
  typedef std::function<bool(const std::string &, S &)> CallFunction;

  ServiceClientMock(const std::string &name, const CallFunction& call_callback)
        : name_(name)
        , call_callback_(call_callback)
  {}

  bool call(S &s)
  {
    return call_callback_(name_, s);
  }

private:
  std::string name_;
  CallFunction call_callback_;

};

/**
 * @brief ...
 */
template <typename S>
class ServiceClientMockFactory
{
public:
  ServiceClientMock<S> create(const std::string &name)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    return ServiceClientMock<S>(name, std::bind(&ServiceClientMockFactory::call_named, this, _1, _2));
  }

  MOCK_CONST_METHOD2_T(call_named, bool(const std::string &name, S& s));
};

TEST(STOExecutorTest, testDummy)
{
  // for ros::Time functionality
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