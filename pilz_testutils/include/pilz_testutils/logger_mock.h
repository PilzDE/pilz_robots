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

#include <gmock/gmock.h>

#include <pilz_testutils/mock_appender.h>

namespace pilz_testutils
{
using std::placeholders::_1;
using std::placeholders::_2;

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
 * StrictMock<pilz_testutils::LoggerMock> ros_log_mock;
 *
 * EXPECT_WARN_LOG(mock_logger, "Your warning text")
 *            .WillOnce(ACTION_OPEN_BARRIER_VOID(LOG_MSG_RECEIVED_EVENT));
 *
 * EXPECT_ARBITRARY_LOG(ros_log_mock).Times(AnyNumber()); // Ensure that arbitrary other logs can occur
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

  //! @deprecated DO NOT USE ANYMORE!, this function solely exist to ensure compatibility with old code
  //! not yet adapted.
  LoggerMock& operator*();

public:
  MOCK_METHOD2(appendLogMsg, void(const log4cxx::spi::LoggingEventPtr&, log4cxx::helpers::Pool&));

private:
  log4cxx::LoggerPtr ros_root_logger_;
  // Note:
  // The ROS root logger takes control over the life time management of the LoggerMock!
  // We only keep a pointer to allow tests make use of the mocked functions of the LoggerMock.
  MockAppender* mock_appender_{ new MockAppender(std::bind(&LoggerMock::appendLogMsg, this, _1, _2)) };
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

inline LoggerMock& LoggerMock::operator*()
{
  return *this;
}

// clang-format off
#define GENERATE_LOGMESSAGE_MATCHER_P(level)\
  MATCHER_P(Is##level, msg, "")\
  {\
    return arg->getLevel()->toInt() == log4cxx::Level::level##_INT && std::string(msg) == arg->getMessage();\
  }

GENERATE_LOGMESSAGE_MATCHER_P(DEBUG)
GENERATE_LOGMESSAGE_MATCHER_P(INFO)
GENERATE_LOGMESSAGE_MATCHER_P(WARN)
GENERATE_LOGMESSAGE_MATCHER_P(ERROR)
GENERATE_LOGMESSAGE_MATCHER_P(FATAL)

//! @deprecated DO NOT USE ANYMORE!, this function solely exist to ensure compatibility with old code
//! not yet adapted.
#define EXPECT_LOG(logger, level, msg) EXPECT_CALL(logger, appendLogMsg(Is##level(msg), ::testing::_))

#define EXPECT_DEBUG_LOG(logger, msg)   EXPECT_CALL(logger, appendLogMsg(IsDEBUG(msg),  ::testing::_))
#define EXPECT_INFO_LOG(logger, msg)    EXPECT_CALL(logger, appendLogMsg(IsINFO(msg),   ::testing::_))
#define EXPECT_WARN_LOG(logger, msg)    EXPECT_CALL(logger, appendLogMsg(IsWARN(msg),   ::testing::_))
#define EXPECT_ERROR_LOG(logger, msg)   EXPECT_CALL(logger, appendLogMsg(IsERROR(msg),  ::testing::_))
#define EXPECT_FATAL_LOG(logger, msg)   EXPECT_CALL(logger, appendLogMsg(IsFATAL(msg),  ::testing::_))

#define EXPECT_ARBITRARY_LOG(logger)    EXPECT_CALL(logger, appendLogMsg(::testing::_,  ::testing::_))
// clang-format on

}  // namespace pilz_testutils

#endif  // LOGGER_MOCK_H
