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
#ifndef ROS_LOG_EXTENDER_H
#define ROS_LOG_EXTENDER_H

#include <string>

#include <log4cxx/logger.h>

#include <pilz_testutils/logger_mock.h>

namespace pilz_testutils
{
/**
 * @brief Class for checking ROS log messages during tests.
 *
 *
 * \e Usage:<br>
 * Suppose you expect a certain warning during your test use
 *
 * \code
 *
 * #include <pilz_testutils/async_test.h>
 * #include <pilz_testutils/ros_log_extender.h>
 *
 *
 * const std::string LOG_MSG_RECEIVED_EVENT{ "logger_called_event" };
 *
 * pilz_testutils::ROSLogExtender ros_log_mock;
 *
 * EXPECT_CALL(*ros_log_mock, append(IsWarn("Your warning text"),_))
 *            .WillOnce(ACTION_OPEN_BARRIER_VOID(LOG_MSG_RECEIVED_EVENT));
 *
 * function_causing_warning();
 *
 * BARRIER(LOG_MSG_RECEIVED_EVENT); // Wait till log message is received
 *
 * \endcode
 */
class ROSLogExtender
{
public:
  ROSLogExtender(const std::string& logger_name = ROSCONSOLE_ROOT_LOGGER_NAME);
  ~ROSLogExtender();

public:
  LoggerMock& operator*();

private:
  log4cxx::LoggerPtr ros_root_logger_;
  // Note:
  // The ROS root logger takes control over the life time management of the LoggerMock!
  // We only keep a pointer to allow tests make use of the mocked functions of the LoggerMock.
  LoggerMock* logger_mock_{ new LoggerMock() };
};

inline ROSLogExtender::ROSLogExtender(const std::string& logger_name)
  : ros_root_logger_(log4cxx::Logger::getLogger(logger_name))
{
  ros_root_logger_->addAppender(logger_mock_);
}

inline ROSLogExtender::~ROSLogExtender()
{
  ros_root_logger_->removeAppender(logger_mock_);
}

inline LoggerMock& ROSLogExtender::operator*()
{
  return *logger_mock_;
}

}  // namespace pilz_testutils

#endif  // ROS_LOG_EXTENDER_H
