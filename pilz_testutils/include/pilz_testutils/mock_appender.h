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

#include <gmock/gmock.h>

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
  ~MockAppender()
  {
  }

public:
  MOCK_METHOD2(append, void(const log4cxx::spi::LoggingEventPtr&, log4cxx::helpers::Pool&));

  log4cxx::LogString getName() const override
  {
    return "MockAppender";
  }

  virtual void close()
  {
  }
  virtual bool requiresLayout() const
  {
    return false;
  }
};

#define GENERATE_LOGMESSAGE_MATCHER_P(level)                                                                           \
  MATCHER_P(Is##level, msg, std::string(#level " \"") + msg + "\"")                                                    \
  {                                                                                                                    \
    return arg->getLevel()->toInt() == log4cxx::Level::level##_INT && std::string(msg) == arg->getMessage();           \
  }

GENERATE_LOGMESSAGE_MATCHER_P(DEBUG)
GENERATE_LOGMESSAGE_MATCHER_P(INFO)
GENERATE_LOGMESSAGE_MATCHER_P(WARN)
GENERATE_LOGMESSAGE_MATCHER_P(ERROR)
GENERATE_LOGMESSAGE_MATCHER_P(FATAL)

constexpr bool str_eq(char const* str1, char const* str2)
{
  return *str1 == *str2 && (*str1 == '\0' || str_eq(str1 + 1, str2 + 1));
}

#define ASSERT_VALID_LEVEL(level)                                                                                      \
  static_assert(pilz_testutils::str_eq(#level, "DEBUG") || pilz_testutils::str_eq(#level, "INFO") ||                   \
                    pilz_testutils::str_eq(#level, "WARN") || pilz_testutils::str_eq(#level, "ERROR") ||               \
                    pilz_testutils::str_eq(#level, "FATAL"),                                                           \
                "\"" #level "\" is not a valid log level");

MATCHER_P(IsLevel, level, std::string("Level is: ") + level->toString())
{
  return arg->getLevel() == level;
}

MATCHER_P(IsMessage, msg, std::string("Message is: ") + msg)
{
  return arg->getMessage() == msg;
}

// The ASSERT_VALID_LEVEL avoids accidentally defaulting of log4cxx::Level::toLevel() to DEBUG due to missspelled level
#define EXPECT_LOG(logger, level, msg)                                                                                 \
  ASSERT_VALID_LEVEL(level)                                                                                            \
  EXPECT_CALL(logger, append(::testing::AllOf(pilz_testutils::IsLevel(log4cxx::Level::toLevel(#level)),                \
                                              pilz_testutils::IsMessage(msg)),                                         \
                             ::testing::_))

}  // namespace pilz_testutils

namespace log4cxx::spi
{
void PrintTo(const LoggingEventPtr& logging_event, std::ostream* os)
{
  *os << logging_event->getLevel()->toString() << " \"" << logging_event->getMessage() << "\"";
}
}  // namespace log4cxx::spi

#endif  // MOCK_APPENDER_H