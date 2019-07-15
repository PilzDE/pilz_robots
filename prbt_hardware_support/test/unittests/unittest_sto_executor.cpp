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

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <pilz_testutils/async_test.h>

#include <prbt_hardware_support/sto_executor.h>
#include <prbt_hardware_support/service_client_mock.h>

#define EXPECT_RECOVER                                 \
  EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _)) \
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(RECOVER_SRV_CALLED_EVENT), Return(true)))

#define EXPECT_UNHOLD                                  \
  EXPECT_CALL(mock_factory_, call_named(UNHOLD_SERVICE, _)) \
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(UNHOLD_SRV_CALLED_EVENT), Return(true)))

#define EXPECT_HOLD                                 \
  EXPECT_CALL(mock_factory_, call_named(HOLD_SERVICE, _)) \
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(HOLD_SRV_CALLED_EVENT), Return(true)))

#define EXPECT_HALT                                  \
  EXPECT_CALL(mock_factory_, call_named(HALT_SERVICE, _)) \
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(HALT_SRV_CALLED_EVENT), Return(true)))

namespace prbt_hardware_support_tests
{

using namespace prbt_hardware_support;

using ::testing::_;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::InvokeWithoutArgs;
using ::testing::Return;

typedef ServiceClientMockFactory<std_srvs::Trigger> MockFactory;
typedef STOExecutorTemplated<ServiceClientMock<std_srvs::Trigger>> STOExecutor;

const std::string RECOVER_SRV_CALLED_EVENT{"recover_srv_called"};
const std::string UNHOLD_SRV_CALLED_EVENT{"unhold_srv_called"};
const std::string HOLD_SRV_CALLED_EVENT{"hold_srv_called"};
const std::string HALT_SRV_CALLED_EVENT{"halt_srv_called"};

class STOExecutorTest : public ::testing::Test, public ::testing::AsyncTest
{
protected:
  MockFactory mock_factory_;
};

/**
 * @brief Test D0 destructor
 *
 * Increases function coverage
 */
TEST_F(STOExecutorTest, testD0estructor)
{
  std::shared_ptr<STOExecutor> sto_executor{new STOExecutor(std::bind(&MockFactory::create, &mock_factory_,
                                                                      std::placeholders::_1))};
}

/**
 * @brief Checks if a running sto executor can be terminated.
 */
TEST_F(STOExecutorTest, testTerminateThread)
{
  STOExecutor sto_executor{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};
  sto_executor.runAsync();
  sto_executor.terminate();
}

/**
 * @brief Checks if a running sto executor is terminated for destruction.
 */
TEST_F(STOExecutorTest, testTerminateThreadAtDestruction)
{
  STOExecutor sto_executor{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};
  sto_executor.runAsync();
}

/**
 * @brief Test enabling
 *
 * Test Sequence:
 *  1. Run the sto executor and call update(true),
 *     let recover service return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 */
TEST_F(STOExecutorTest, testEnable)
{
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  STOExecutor sto_executor{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};
  sto_executor.runAsync();

  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);
}

/**
 * @brief Test enabling, disabling and enabling again
 *
 * Test Sequence:
 *  1. Run the sto executor and call update(true),
 *     let recover and unhold services return success
 *  2. Call update(false),
 *     let hold and halt services return success
 *  3. Call update(true),
 *     let recover service return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold and halt services are called successively
 *  3. Recover and unhold services are called successively
 */
TEST_F(STOExecutorTest, testEnableDisableEnable)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  STOExecutor sto_executor{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};
  sto_executor.runAsync();

  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_HALT;
  }

  sto_executor.update(false);

  BARRIER(HOLD_SRV_CALLED_EVENT);
  BARRIER(HALT_SRV_CALLED_EVENT);

  /**********
   * Step 3 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);
}

/**
 * @brief Test spaming enable plus subsequent disable
 *
 * Test Sequence:
 *  1. Run the sto executor and call update(true) repeatedly,
 *     let recover and unhold services return success
 *  2. Call update(false),
 *     let hold and halt services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold and halt services are called successively
 */
TEST_F(STOExecutorTest, testSpamEnablePlusDisable)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  STOExecutor sto_executor{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};
  sto_executor.runAsync();

  std::atomic_bool keep_spamming{true};
  std::thread spam_enable{[&sto_executor, &keep_spamming]() { while (keep_spamming) { sto_executor.update(true); } }};

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);

  keep_spamming = false;
  spam_enable.join();

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_HALT;
  }

  sto_executor.update(false);

  BARRIER(HOLD_SRV_CALLED_EVENT);
  BARRIER(HALT_SRV_CALLED_EVENT);
}

/**
 * @brief Test spamming disable plus subsequent enable
 *
 * Test Sequence:
 *  1. Run the sto executor and call update(true),
 *     let recover and unhold services return success
 *  2. Call update(false) repeatedly,
 *     let hold and halt services return success
 *  3. Call update(true),
 *     let recover and unhold services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold and halt services are called successively
 *  3. Recover and unhold services are called successively
 */
TEST_F(STOExecutorTest, testSpamDisablePlusEnable)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  STOExecutor sto_executor{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};
  sto_executor.runAsync();

  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_HALT;
  }

  std::atomic_bool keep_spamming{true};
  std::thread spam_enable{[&sto_executor, &keep_spamming]() { while (keep_spamming) { sto_executor.update(false); } }};

  BARRIER(HOLD_SRV_CALLED_EVENT);
  BARRIER(HALT_SRV_CALLED_EVENT);

  keep_spamming = false;
  spam_enable.join();

  /**********
   * Step 3 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);
}

/**
 * @brief Test skipping hold when sto changes to false during recover. Test also a following enabling.
 *
 * Test Sequence:
 *  1. Run the sto executor and call update(true),
 *     call update(false) during recover service call and return success
 *  2. Call update(true),
 *     let recover and unhold services return success
 *
 * Expected Results:
 *  1. Recover and halt services are called successively
 *  2. Recover and unhold services are called successively
 */
TEST_F(STOExecutorTest, testSkippingHoldPlusEnable)
{
  STOExecutor sto_executor{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};

  // define function for recover-invoke action
  std::function<bool()> recover_action = [this, &sto_executor]() {
    this->triggerClearEvent(RECOVER_SRV_CALLED_EVENT);
    sto_executor.update(false);
    return true;
  };

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _))
        .WillOnce(InvokeWithoutArgs(recover_action));

    EXPECT_HALT;
  }

  sto_executor.runAsync();
  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(HALT_SRV_CALLED_EVENT);

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);
}

/**
 * @brief Test skipping halt when sto changes to true during hold. Test also a following disable.
 *
 * Test Sequence:
 *  1. Run the sto executor and call update(true),
 *     let recover and unhold services return success
 *  2. Call update(false),
 *     call update(true) during hold service call and return success
 *  1. Call update(false),
 *     let hold and halt services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold and unhold services are called successively
 *  3. Recover and unhold services are called successively
 */
TEST_F(STOExecutorTest, testSkippingHaltPlusEnableDisable)
{
  STOExecutor sto_executor{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};

  // define function for hold-invoke action
  std::function<bool()> hold_action = [this, &sto_executor]() {
    this->triggerClearEvent(HOLD_SRV_CALLED_EVENT);
    sto_executor.update(true);
    return true;
  };

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  sto_executor.runAsync();
  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(mock_factory_, call_named(HOLD_SERVICE, _))
        .WillOnce(InvokeWithoutArgs(hold_action));

    EXPECT_UNHOLD;
  }

  sto_executor.update(false);

  BARRIER(HOLD_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);

  /**********
   * Step 3 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_HALT;
  }

  sto_executor.update(false);

  BARRIER(HOLD_SRV_CALLED_EVENT);
  BARRIER(HALT_SRV_CALLED_EVENT);
}

/**
 * @brief Test enabling with failing recover service and retry (disable plus enable).
 *
 * Test Sequence:
 *  1. Run the sto executor and call update(true),
 *     let recover service fail repeatedly
 *  2. Call update(false) and update(true),
 *     let recover service return success
 *
 * Expected Results:
 *  1. Recover service is called at least once
 *  2. Recover and unhold services are called successively.
 */
TEST_F(STOExecutorTest, testRecoverFailPlusDisable)
{
  /**********
   * Step 1 *
   **********/
  EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _))
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(RECOVER_SRV_CALLED_EVENT), Return(false)))
      .WillRepeatedly(Return(false));

  STOExecutor sto_executor{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};
  sto_executor.runAsync();

  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  sto_executor.update(false);
  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);
}

/**
 * @brief Test enabling with failing unhold service and retry (disable plus enable).
 *
 * Test Sequence:
 *  1. Run the sto executor and call update(true),
 *     let recover service return success and unhold service fail repeatedly
 *  2. Call update(false) and update(true),
 *     let halt, recover and unhold services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively, the latter one at least once
 *  2. Unhold service is called
 */
TEST_F(STOExecutorTest, testUnholdFail)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_RECOVER;

    EXPECT_CALL(mock_factory_, call_named(UNHOLD_SERVICE, _))
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(UNHOLD_SRV_CALLED_EVENT), Return(false)))
      .WillRepeatedly(Return(false));
  }

  STOExecutor sto_executor{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};
  sto_executor.runAsync();

  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(mock_factory_, call_named(HALT_SERVICE, _))
      .WillRepeatedly(Return(true));
    EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _))
      .WillRepeatedly(Return(true));

    EXPECT_UNHOLD;
  }

  sto_executor.update(false);
  sto_executor.update(true);

  BARRIER(UNHOLD_SRV_CALLED_EVENT);
}

/**
 * @brief Test disabling with failing hold service and subsequent enabling
 *
 * Test Sequence:
 *  1. Run the sto executor and call update(true),
 *     let recover and unhold services return success
 *  2. Call update(false),
 *     let hold services fail and halt service return success
 *  3. Call update(true),
 *     let recover and unhold services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold service is called
 *  3. Recover and unhold services are called successively
 */
TEST_F(STOExecutorTest, testHoldFail)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  STOExecutor sto_executor{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};
  sto_executor.runAsync();

  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(mock_factory_, call_named(HOLD_SERVICE, _))
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(HOLD_SRV_CALLED_EVENT), Return(false)))
      .WillRepeatedly(Return(false));

    EXPECT_CALL(mock_factory_, call_named(HALT_SERVICE, _))
      .WillRepeatedly(Return(true));
  }

  sto_executor.update(false);

  BARRIER(HOLD_SRV_CALLED_EVENT);

  /**********
   * Step 3 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);
}

/**
 * @brief Test disabling with failing halt service and subsequent enabling
 *
 * Test Sequence:
 *  1. Run the sto executor and call update(true),
 *     let recover and unhold services return success
 *  2. Call update(false),
 *     let hold service return success and halt service fail repeatedly
 *  3. Call update(true),
 *     let recover and unhold services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold and halt services are called successively
 *  3. Recover and unhold services are called successively
 */
TEST_F(STOExecutorTest, testHaltFail)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  STOExecutor sto_executor{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};
  sto_executor.runAsync();

  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;

    EXPECT_HOLD;

    EXPECT_CALL(mock_factory_, call_named(HALT_SERVICE, _))
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(HALT_SRV_CALLED_EVENT), Return(false)))
      .WillRepeatedly(Return(false));
  }

  sto_executor.update(false);

  BARRIER(HOLD_SRV_CALLED_EVENT);
  BARRIER(HALT_SRV_CALLED_EVENT);

  /**********
   * Step 3 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  sto_executor.update(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
  BARRIER(UNHOLD_SRV_CALLED_EVENT);
}

} // namespace prbt_hardware_support_tests

int main(int argc, char **argv)
{
  // for (limited) ros::Time functionality, no ROS communication
  ros::Time::init();

  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}