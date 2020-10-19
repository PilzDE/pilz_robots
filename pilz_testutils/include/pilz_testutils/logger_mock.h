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
#ifndef LOGGER_MOCK_H
#define LOGGER_MOCK_H

#include <string>

#include <log4cxx/logger.h>

#include <pilz_testutils/mock_appender.h>

namespace pilz_testutils
{
/**
 * @brief Class to be used in tests with logging checks.
 *
 * With deletion of this class the internal logging mechanism is detached.
 *
 * <b>Usage</b><br>
 *
 * \code
 *
 * #include <pilz_testutils/logger_mock.h>
 *
 * pilz_testutils::LoggerMock ros_log_mock;
 *
 * EXPECT_LOG(*mock_logger, WARN, "Your warning text").Times(1);
 *
 * function_causing_warning();
 *
 * \endcode
 * <br>
 * <b>Asynchronous usage in combination with testing::AsyncTest</b><br>
 *
 * \code
 *
 * #include <pilz_testutils/async_test.h>
 * #include <pilz_testutils/logger_mock.h>
 *
 *
 * const std::string LOG_MSG_RECEIVED_EVENT{ "logger_called_event" };
 *
 * pilz_testutils::LoggerMock ros_log_mock;
 *
 * EXPECT_LOG(*mock_logger, WARN, "Your warning text")
 *            .WillOnce(ACTION_OPEN_BARRIER_VOID(LOG_MSG_RECEIVED_EVENT));
 *
 * function_causing_async_warning();
 *
 * BARRIER(LOG_MSG_RECEIVED_EVENT); // Wait till log message is received
 *
 * \endcode
 */
class LoggerMock
{
public:
  LoggerMock(const std::string& logger_name = ROSCONSOLE_ROOT_LOGGER_NAME);
  ~LoggerMock();

public:
  MockAppender& operator*();

private:
  log4cxx::LoggerPtr ros_root_logger_;
  // Note:
  // The ROS root logger takes control over the life time management of the LoggerMock!
  // We only keep a pointer to allow tests make use of the mocked functions of the LoggerMock.
  MockAppender* mock_appender_{ new MockAppender() };
};

inline LoggerMock::LoggerMock(const std::string& logger_name)
  : ros_root_logger_(log4cxx::Logger::getLogger(logger_name))
{
  ros_root_logger_->addAppender(mock_appender_);
}

inline LoggerMock::~LoggerMock()
{
  ros_root_logger_->removeAppender(mock_appender_);
}

inline MockAppender& LoggerMock::operator*()
{
  return *mock_appender_;
}

}  // namespace pilz_testutils

#endif  // LOGGER_MOCK_H
