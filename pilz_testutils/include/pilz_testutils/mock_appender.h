/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
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

#ifndef MOCK_APPENDER_H
#define MOCK_APPENDER_H

#include <stdexcept>
#include <functional>

#include <ros/console.h>
#include "ros/console_impl.h"

#include "log4cxx/appenderskeleton.h"
#include "log4cxx/level.h"

namespace pilz_testutils
{
/**
 * @brief Mocked version of a log4cxx::Appender to be used by gtest
 *
 * @related LoggerMock
 *
 * @note Not intended for direct use. Please use LoggerMock inside the tests.
 *
 */
class MockAppender : public log4cxx::AppenderSkeleton
{
public:
  using AppendCallback = std::function<void(const log4cxx::spi::LoggingEventPtr& a, log4cxx::helpers::Pool& b)>;

  MockAppender(const AppendCallback& append_cb) : append_cb_(append_cb)
  {
    if (!append_cb)
    {
      throw std::invalid_argument("Append callback must not be null");
    }
  }

public:
  ~MockAppender() = default;

public:
  void append(const log4cxx::spi::LoggingEventPtr& a, log4cxx::helpers::Pool& b) override
  {
    append_cb_(a, b);
  }

  log4cxx::LogString getName() const override
  {
    return "MockAppender";
  }

  void close() override
  {
  }
  bool requiresLayout() const override
  {
    return false;
  }

private:
  AppendCallback append_cb_;
};

}  // namespace pilz_testutils

#endif  // MOCK_APPENDER_H
