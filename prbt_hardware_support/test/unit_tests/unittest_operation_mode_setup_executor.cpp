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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <ros/time.h>

#include <pilz_msgs/GetSpeedOverride.h>

#include <prbt_hardware_support/OperationModes.h>
#include <prbt_hardware_support/operation_mode_setup_executor.h>
#include <prbt_hardware_support/operation_mode_setup_executor_node_service_calls.h>

namespace operation_mode_setup_executor_tests
{
using namespace prbt_hardware_support;

using ::testing::_;
using ::testing::Le;
using ::testing::Return;
using ::testing::SetArgReferee;
using ::testing::DoAll;



class OperationModeSetupExecutorTest : public ::testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

  MOCK_METHOD1(setSpeedLimit, bool(const double &));

public:
  double t1_limit_{ 0.1 };
  double auto_limit_{ 0.2 };
  std::unique_ptr<OperationModeSetupExecutor> executor_;
};

void OperationModeSetupExecutorTest::SetUp()
{
  ros::Time::init();

  OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = OperationModes::UNKNOWN;

  ON_CALL(*this, setSpeedLimit(_)).WillByDefault(Return(true));

  executor_ = std::unique_ptr<OperationModeSetupExecutor>(new OperationModeSetupExecutor(
                                                            t1_limit_,
                                                            auto_limit_,
                                                            std::bind(&OperationModeSetupExecutorTest::setSpeedLimit, this, std::placeholders::_1)));
}

void OperationModeSetupExecutorTest::TearDown()
{
  ros::Time::shutdown();
}

/**
 * @tests{Speed_limits_per_operation_mode,
 * Tests that speed limit is set according to current operation mode.
 * }
 *
 * Test Sequence:
 *  1. Call updateOperationMode() with operation mode T1 and current time stamp.
 *  2. Call updateOperationMode() with operation mode AUTO and current time stamp.
 *
 * Expected Results:
 *  1. setSpeedLimit() is called with predefined T1 limit.
 *  2. setSpeedLimit() is called with predefined AUTO limit.
 */
TEST_F(OperationModeSetupExecutorTest, testUpdateOperationMode)
{
  /**********
   * Step 1 *
   **********/
  OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = OperationModes::T1;

  EXPECT_CALL(*this, setSpeedLimit(t1_limit_)).WillOnce(Return(true));

  executor_->updateOperationMode(op_mode);

  /**********
   * Step 2 *
   **********/
  OperationModes op_mode2;
  op_mode2.time_stamp = ros::Time::now();
  op_mode2.value = OperationModes::AUTO;

  EXPECT_CALL(*this, setSpeedLimit(auto_limit_)).WillOnce(Return(true));

  executor_->updateOperationMode(op_mode2);
}

/**
 * @tests{Speed_limits_per_operation_mode,
 * Test updateOperationMode() with no change in operation mode.
 * }
 *
 * Test Sequence:
 *  1. Call updateOperationMode() with operation mode T1 and current time stamp.
 *  2. Call updateOperationMode() with operation mode T1 and current time stamp.
 *
 * Expected Results:
 *  1. setSpeedLimit() is called with predefined T1 limit.
 *  2. setSpeedLimit() can be called with predefined T1 limit.
 */
TEST_F(OperationModeSetupExecutorTest, testUpdateOperationModeSameMode)
{
  /**********
   * Step 1 *
   **********/
  OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = OperationModes::T1;

  EXPECT_CALL(*this, setSpeedLimit(t1_limit_)).WillOnce(Return(true));

  executor_->updateOperationMode(op_mode);

  /**********
   * Step 2 *
   **********/
  OperationModes op_mode2;
  op_mode2.time_stamp = ros::Time::now();
  op_mode2.value = op_mode.value;

  EXPECT_CALL(*this, setSpeedLimit(t1_limit_)).WillRepeatedly(Return(true));

  executor_->updateOperationMode(op_mode2);
}

/**
 * @tests{Speed_limits_per_operation_mode,
 * Test updateOperationMode() with no change in time.
 * }
 *
 * Test Sequence:
 *  1. Call updateOperationMode() with operation mode T1 and current time stamp.
 *  2. Call updateOperationMode() with operation mode AUTO and previous time stamp.
 *
 * Expected Results:
 *  1. setSpeedLimit() is called with predefined T1 limit.
 *  2. setSpeedLimit() is not called.
 */
TEST_F(OperationModeSetupExecutorTest, testUpdateOperationModeSameTime)
{
  /**********
   * Step 1 *
   **********/
  OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = OperationModes::T1;

  EXPECT_CALL(*this, setSpeedLimit(t1_limit_)).WillOnce(Return(true));

  executor_->updateOperationMode(op_mode);

  /**********
   * Step 2 *
   **********/
  OperationModes op_mode2;
  op_mode2.time_stamp = op_mode.time_stamp;
  op_mode2.value = OperationModes::AUTO;

  EXPECT_CALL(*this, setSpeedLimit(auto_limit_)).Times(0);

  executor_->updateOperationMode(op_mode2);
}

/**
 * @tests{Speed_limits_per_operation_mode,
 * Test updateOperationMode() with unknown operation mode.
 * }
 *
 * Test Sequence:
 *  1. Call updateOperationMode() with operation mode UNKNOWN and current time stamp.
 *
 * Expected Results:
 *  1. setSpeedLimit() is called with limit less or equal 0.0.
 */
TEST_F(OperationModeSetupExecutorTest, testUpdateOperationModeUnknownMode)
{
  /**********
   * Step 1 *
   **********/
  OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = OperationModes::UNKNOWN;

  EXPECT_CALL(*this, setSpeedLimit(Le(0.0))).WillOnce(Return(true));

  executor_->updateOperationMode(op_mode);
}

class SetSpeedLimitServiceMock
{
public:
  MOCK_METHOD1(call, bool(SetSpeedLimit& srv));
  MOCK_METHOD0(getService, std::string());
};

MATCHER_P(IsCorrectSpeedLimitSet, speed_limit, "") { return arg.request.speed_limit == speed_limit; }

/**
 * @brief Tests the correct behavior in case the SetSpeedLimit service
 * succeeds.
 */
TEST_F(OperationModeSetupExecutorTest, testSetSpeedLimitSrvSuccess)
{
  const double exp_limit {7.7};
  SetSpeedLimit exp_srv;
  exp_srv.request.speed_limit = exp_limit;

  SetSpeedLimitServiceMock mock;
  EXPECT_CALL(mock, getService()).WillRepeatedly(Return("TestServiceName"));
  EXPECT_CALL(mock, call(IsCorrectSpeedLimitSet(exp_limit))).Times(1).WillOnce(Return(true));

  EXPECT_TRUE(setSpeedLimitSrv<SetSpeedLimitServiceMock>(mock, exp_limit));
}

/**
 * @brief Tests the correct behavior in case the SetSpeedLimit service fails.
 */
TEST_F(OperationModeSetupExecutorTest, testSetSpeedLimitSrvFailure)
{
  SetSpeedLimitServiceMock mock;
  EXPECT_CALL(mock, getService()).WillRepeatedly(Return("TestServiceName"));
  EXPECT_CALL(mock, call(_)).Times(1).WillOnce(Return(false));

  const double exp_limit {7.7};
  EXPECT_FALSE(setSpeedLimitSrv<SetSpeedLimitServiceMock>(mock, exp_limit));
}

class OperationModeSetupExecutorTestSpeedOverride : public OperationModeSetupExecutorTest,
                                  public ::testing::WithParamInterface<std::pair<OperationModes::_value_type, double>>
{
  OperationModes::_value_type getMode()
  {
    return GetParam().first;
  }

  double getSpeedOverride()
  {
    return GetParam().second;
  }
};

// Usage: If you don't care about uninteresting calls the the MOCK_METHOD but want to suppress the warning
typedef ::testing::NiceMock<OperationModeSetupExecutorTestSpeedOverride>
  OperationModeSetupExecutorTestSpeedOverrideNice;

/**
 * @tests{speed_override_per_operation_mode,
 * Tests that speed override is set according to current operation mode.
 * }
 *
 */
TEST_P(OperationModeSetupExecutorTestSpeedOverrideNice, testSpeedOverride)
{
  /**********
   * Step 1 *
   **********/
  OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = GetParam().first;

  executor_->updateOperationMode(op_mode);

  auto req = pilz_msgs::GetSpeedOverrideRequest();
  auto res = pilz_msgs::GetSpeedOverrideResponse();
  executor_->getSpeedOverride(req, res);
  EXPECT_EQ(res.speed_override, GetParam().second);
}

INSTANTIATE_TEST_CASE_P(
  SpeedOverrideModeTests,
  OperationModeSetupExecutorTestSpeedOverrideNice,
  ::testing::Values(
    std::pair<OperationModes::_value_type, double>(OperationModes::UNKNOWN, 0.0),
    std::pair<OperationModes::_value_type, double>(OperationModes::T1,      0.1),
    std::pair<OperationModes::_value_type, double>(OperationModes::AUTO,    1.0)
  ),
);

}  // namespace operation_mode_setup_executor_tests

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
