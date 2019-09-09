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

#include <prbt_hardware_support/OperationModes.h>
#include <prbt_hardware_support/operation_mode_setup_executor.h>

namespace operation_mode_setup_executor_tests
{
using namespace prbt_hardware_support;

using ::testing::_;
using ::testing::Le;
using ::testing::Return;

class OperationModeSetupExecutorTest : public ::testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

  MOCK_METHOD1(setSpeedLimit, bool(const double &));
  MOCK_METHOD0(getOperationMode, OperationModes());

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

  ON_CALL(*this, getOperationMode()).WillByDefault(Return(op_mode));
  ON_CALL(*this, setSpeedLimit(_)).WillByDefault(Return(true));

  executor_ = std::unique_ptr<OperationModeSetupExecutor>(new OperationModeSetupExecutor(
      t1_limit_,
      auto_limit_,
      std::bind(&OperationModeSetupExecutorTest::setSpeedLimit, this, std::placeholders::_1),
      std::bind(&OperationModeSetupExecutorTest::getOperationMode, this)));
}

void OperationModeSetupExecutorTest::TearDown()
{
  ros::Time::shutdown();
}

/**
 * Test constructor of OperationModeSetupExecutor with operation mode T1.
 */
TEST_F(OperationModeSetupExecutorTest, testConstructor)
{
  OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = OperationModes::T1;

  EXPECT_CALL(*this, getOperationMode()).WillOnce(Return(op_mode));
  EXPECT_CALL(*this, setSpeedLimit(_)).WillOnce(Return(true));

  OperationModeSetupExecutor executor(
      t1_limit_,
      auto_limit_,
      std::bind(&OperationModeSetupExecutorTest::setSpeedLimit, this, std::placeholders::_1),
      std::bind(&OperationModeSetupExecutorTest::getOperationMode, this));
}

/**
 * Test updateOperationMode().
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
 * Test updateOperationMode() with no change in operation mode.
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
 * Test updateOperationMode() with no change in time.
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
 * Test updateOperationMode() with unknown operation mode
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

}  // namespace operation_mode_setup_executor_tests

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
