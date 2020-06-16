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
#ifndef SCOPED_LOGGER_MOCK_HOLDER_H
#define SCOPED_LOGGER_MOCK_HOLDER_H

#include <string>

#include <log4cxx/logger.h>

#include <pilz_testutils/logger_mock.h>

namespace pilz_testutils
{
/**
 * @brief Class holding a logger mock within a given scope to be used in tests with logging checks.
 *
 * This class serves as scoped version of the LoggerMock.
 * After going out of scoped the LoggerMock is removed from the logging
 * mechanism.
 *
 * <b>Usage</b><br>
 *
 * \code
 *
 * #include <pilz_testutils/scoped_logger_mock_holder.h>
 *
 * pilz_testutils::ScopedLoggerMockHolder ros_log_mock;
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
 * #include <pilz_testutils/scoped_logger_mock_holder.h>
 *
 *
 * const std::string LOG_MSG_RECEIVED_EVENT{ "logger_called_event" };
 *
 * pilz_testutils::ScopedLoggerMockHolder ros_log_mock;
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
class ScopedLoggerMockHolder
{
public:
  ScopedLoggerMockHolder(const std::string& logger_name = ROSCONSOLE_ROOT_LOGGER_NAME);
  ~ScopedLoggerMockHolder();

public:
  LoggerMock& operator*();

private:
  log4cxx::LoggerPtr ros_root_logger_;
  // Note:
  // The ROS root logger takes control over the life time management of the LoggerMock!
  // We only keep a pointer to allow tests make use of the mocked functions of the LoggerMock.
  LoggerMock* logger_mock_{ new LoggerMock() };
};

inline ScopedLoggerMockHolder::ScopedLoggerMockHolder(const std::string& logger_name)
  : ros_root_logger_(log4cxx::Logger::getLogger(logger_name))
{
  ros_root_logger_->addAppender(logger_mock_);
}

inline ScopedLoggerMockHolder::~ScopedLoggerMockHolder()
{
  ros_root_logger_->removeAppender(logger_mock_);
}

inline LoggerMock& ScopedLoggerMockHolder::operator*()
{
  return *logger_mock_;
}

}  // namespace pilz_testutils

#endif  // SCOPED_LOGGER_MOCK_HOLDER_H
