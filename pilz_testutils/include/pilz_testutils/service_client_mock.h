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

#ifndef PILZ_TESTUTILS_SERVICE_CLIENT_MOCK_H
#define PILZ_TESTUTILS_SERVICE_CLIENT_MOCK_H

#include <functional>
#include <string>

#include <gmock/gmock.h>

namespace pilz_testutils
{
/**
 * @brief Mock for a ros::ServiceClient, calls are passed through to a given callback (which can be a mock method).
 *
 * @tparam S service type
 */
template <typename S>
class ServiceClientMock
{
public:
  typedef std::function<bool(const std::string&, S&)> CallFunction;
  typedef std::function<bool(const std::string&)> LogicalOperatorFunction;

  ServiceClientMock(const std::string& name, const CallFunction& call_callback,
                    const LogicalOperatorFunction& negation_operator_callback)
    : name_(name), call_callback_(call_callback), negation_operator_callback_(negation_operator_callback)
  {
  }

  bool operator!() const
  {
    return negation_operator_callback_(name_);
  }

  bool call(S& s)
  {
    ROS_DEBUG_NAMED("Mock", "Received call to %s", name_.c_str());
    return call_callback_(name_, s);
  }

  std::string getService() const
  {
    return name_;
  }

private:
  std::string name_;
  CallFunction call_callback_;
  LogicalOperatorFunction negation_operator_callback_;
};

/**
 * @brief Provides a mock method for service calls for a specific service type.
 *
 * @tparam S service type
 */
template <typename S>
class ServiceClientMockFactory
{
public:
  /**
   * @brief Returns a ServiceClientMock, which passes service calls to a (named) mock method.
   *
   * Expectations on service calls can be made as follows:
   * @code
   * EXPECT_CALL(srv_client_mock_factory, call_named(srv_name, srv_req_resp))
   * @endcode
   *
   * @param name the service name
   * @param persistent not used by the mock
   */
  ServiceClientMock<S> create(const std::string& name, bool /*persistent*/)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    return ServiceClientMock<S>(name, std::bind(&ServiceClientMockFactory::call_named, this, _1, _2),
                                std::bind(&ServiceClientMockFactory::handle_invalid_named, this, _1));
  }

  // service call mock method
  MOCK_CONST_METHOD2_T(call_named, bool(const std::string& name, S& s));
  MOCK_CONST_METHOD1_T(handle_invalid_named, bool(const std::string& name));
};

}  // namespace pilz_testutils

#endif  // PILZ_TESTUTILS_SERVICE_CLIENT_MOCK_H
