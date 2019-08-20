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
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <pilz_testutils/async_test.h>
#include <pilz_testutils/service_client_mock.h>

#include <prbt_hardware_support/adapter_sto.h>

#define EXPECT_RECOVER                                       \
  EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _)) \
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(RECOVER_SRV_CALLED_EVENT), Return(true)))

#define EXPECT_UNHOLD                                       \
  EXPECT_CALL(mock_factory_, call_named(UNHOLD_SERVICE, _)) \
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(UNHOLD_SRV_CALLED_EVENT), Return(true)))

#define EXPECT_HOLD                                       \
  EXPECT_CALL(mock_factory_, call_named(HOLD_SERVICE, _)) \
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(HOLD_SRV_CALLED_EVENT), Return(true)))

#define EXPECT_HALT                                       \
  EXPECT_CALL(mock_factory_, call_named(HALT_SERVICE, _)) \
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(HALT_SRV_CALLED_EVENT), Return(true)))

namespace prbt_hardware_support_tests
{

using namespace prbt_hardware_support;
using namespace pilz_testutils;

using ::testing::_;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::InvokeWithoutArgs;
using ::testing::Return;

typedef ServiceClientMock<std_srvs::Trigger> ClientMock;
typedef ServiceClientMockFactory<std_srvs::Trigger> MockFactory;

const std::string RECOVER_SRV_CALLED_EVENT{"recover_srv_called"};
const std::string UNHOLD_SRV_CALLED_EVENT{"unhold_srv_called"};
const std::string HOLD_SRV_CALLED_EVENT{"hold_srv_called"};
const std::string HALT_SRV_CALLED_EVENT{"halt_srv_called"};

class AdapterStoTest : public ::testing::Test, public ::testing::AsyncTest
{
protected:
  MockFactory mock_factory_;
};

/**
 * Gives access to protected methods of sto adapter.
 */
class AdapterSto : public AdapterStoTemplated<ClientMock>
{
public:
  AdapterSto(const std::function<ClientMock(std::string, bool)> &create_service_client)
    : AdapterStoTemplated<ClientMock>(create_service_client)
  {
  }

  FRIEND_TEST(AdapterStoTest, testExitInStateEnabling);
  FRIEND_TEST(AdapterStoTest, testExitInStateStopRequestedDuringEnable);
  FRIEND_TEST(AdapterStoTest, testExitDuringPendingHaltCall);
};

const std::string RECOVER_SERVICE{AdapterSto::RECOVER_SERVICE};
const std::string UNHOLD_SERVICE{AdapterSto::UNHOLD_SERVICE};
const std::string HOLD_SERVICE{AdapterSto::HOLD_SERVICE};
const std::string HALT_SERVICE{AdapterSto::HALT_SERVICE};

/**
 * @brief Test D0 destructor
 *
 * Increases function coverage
 */
TEST_F(AdapterStoTest, testD0estructor)
{
  typedef prbt_hardware_support::AdapterStoTemplated<ClientMock> AdapterSto;
  std::shared_ptr<AdapterSto> adapter_sto{new AdapterSto(std::bind(&MockFactory::create,
                                                                   &mock_factory_,
                                                                   std::placeholders::_1,
                                                                   std::placeholders::_2))};
}

/**
 * @brief Test enabling
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     let recover and unhold services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 */
TEST_F(AdapterStoTest, testEnable)
{
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});
}

/**
 * @brief Test enabling, stopping and enabling again
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     let recover and unhold services return success
 *  2. Call updateSto(false)),
 *     let hold and halt services return success
 *  3. Call updateSto(true)) repeatedly,
 *     let recover and unhold service return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold and halt services are called successively
 *  3. Recover and unhold services are called successively
 */
TEST_F(AdapterStoTest, testEnableStopEnable)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_HALT;
  }

  adapter_sto.updateSto(false);

  BARRIER2({HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});

  /**********
   * Step 3 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  std::atomic_bool keep_spamming{true};
  std::thread spam_enable{[&adapter_sto, &keep_spamming]() { while (keep_spamming) { adapter_sto.updateSto(true); } }};

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  keep_spamming = false;
  spam_enable.join();
}

/**
 * @brief Test spaming enable plus subsequent stop
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)) repeatedly,
 *     let recover and unhold services return success
 *  2. Call updateSto(true) once more (this is certainly after unhold and needed for full coverage)
 *  3. Call updateSto(false)),
 *     let hold and halt services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. -
 *  3. Hold and halt services are called successively
 */
TEST_F(AdapterStoTest, testSpamEnablePlusStop)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  std::atomic_bool keep_spamming{true};
  std::thread spam_enable{[&adapter_sto, &keep_spamming]() { while (keep_spamming) { adapter_sto.updateSto(true); } }};

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  keep_spamming = false;
  spam_enable.join();

  /**********
   * Step 2 *
   **********/

  adapter_sto.updateSto(true);

  /**********
   * Step 3 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_HALT;
  }

  adapter_sto.updateSto(false);

  BARRIER2({HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});
}

/**
 * @brief Test spamming STO=false plus subsequent enable
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     let recover and unhold services return success
 *  2. Call updateSto(false)) repeatedly,
 *     let hold and halt services return success
 *  3. Call updateSto(true)) repeatedly,
 *     let recover and unhold services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold and halt services are called successively
 *  3. Recover and unhold services are called successively
 */
TEST_F(AdapterStoTest, testSpamStoActivePlusEnable)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_HALT;
  }

  std::atomic_bool keep_spamming{true};
  std::thread spam_disable{[&adapter_sto, &keep_spamming]() { while (keep_spamming) { adapter_sto.updateSto(false); } }};

  BARRIER2({HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});

  keep_spamming = false;
  spam_disable.join();

  /**********
   * Step 3 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  keep_spamming = true;
  std::thread spam_enable{[&adapter_sto, &keep_spamming]() { while (keep_spamming) { adapter_sto.updateSto(true); } }};

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  keep_spamming = false;
  spam_enable.join();
}

/**
 * @brief Test skipping hold when sto changes to false during recover. Test also a following enabling.
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     call updateSto(false)) during recover service call and return success, let halt service return success
 *  2. Call updateSto(true)),
 *     let recover and unhold services return success
 *
 * Expected Results:
 *  1. Recover and halt services are called successively
 *  2. Recover and unhold services are called successively
 */
TEST_F(AdapterStoTest, testSkippingHoldPlusEnable)
{
  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  // define function for recover-invoke action
  std::function<bool()> sto_false_during_recover_action = [this, &adapter_sto]() {
    this->triggerClearEvent(RECOVER_SRV_CALLED_EVENT);
    adapter_sto.updateSto(false);
    return true;
  };

  std::function<bool()> halt_action = [this]() {
    this->triggerClearEvent(HALT_SRV_CALLED_EVENT);
    return true;
  };

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _))
        .WillOnce(InvokeWithoutArgs(sto_false_during_recover_action));

    EXPECT_CALL(mock_factory_, call_named(HALT_SERVICE, _))
        .WillOnce(InvokeWithoutArgs(halt_action));
  }

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  std::atomic_bool keep_spamming{true};
  std::thread spam_enable{[&adapter_sto, &keep_spamming]() { while (keep_spamming) { adapter_sto.updateSto(true); } }};

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  keep_spamming = false;
  spam_enable.join();
}

/**
 * @brief Test sending sto change to true while halt call is still pending.
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     call updateSto(false)) during recover service call and return success.
 *     Before returning success on halt service call updateSto(true)
 *
 * Expected Results:
 *  1. Recover and halt services are called successively. Afterwards recover and unhold are called.
 */
TEST_F(AdapterStoTest, testEnableDuringHaltService)
{
  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  // define function for recover-invoke action
  std::function<bool()> sto_false_during_recover_action = [this, &adapter_sto]() {
    this->triggerClearEvent(RECOVER_SRV_CALLED_EVENT);
    adapter_sto.updateSto(false);
    return true;
  };

  std::function<bool()> enable_during_halt_action = [this, &adapter_sto]() {
    this->triggerClearEvent(HALT_SRV_CALLED_EVENT);
    adapter_sto.updateSto(true);
    return true;
  };

  /**********
   * Step 1 *
   **********/

  const std::string RECOVER_SRV_CALLED_EVENT2{"recover_srv_called2"};
  const std::string UNHOLD_SRV_CALLED_EVENT2{"unhold_srv_called2"};
  {
    InSequence dummy;

    EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _))
        .WillOnce(InvokeWithoutArgs(sto_false_during_recover_action));

    EXPECT_CALL(mock_factory_, call_named(HALT_SERVICE, _))
        .WillOnce(InvokeWithoutArgs(enable_during_halt_action));

    EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _))
        .WillOnce(InvokeWithoutArgs([this, RECOVER_SRV_CALLED_EVENT2]() {
          this->triggerClearEvent(RECOVER_SRV_CALLED_EVENT2);
          return true;
        }));

    EXPECT_CALL(mock_factory_, call_named(UNHOLD_SERVICE, _))
        .WillOnce(InvokeWithoutArgs([this, UNHOLD_SRV_CALLED_EVENT2]() {
          this->triggerClearEvent(UNHOLD_SRV_CALLED_EVENT2);
          return true;
        }));
  }

  adapter_sto.updateSto(true);

  BARRIER({RECOVER_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT, RECOVER_SRV_CALLED_EVENT2, UNHOLD_SRV_CALLED_EVENT2});
}

/**
 * @brief Test sending sto change to true and back to false while halt call is still pending.
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)).
 *     Call updateSto(false)) during recover service call and return success. (This is not essential for the test)
 *     Before returning success on halt service call updateSto(true) and update(false).
 *
 * Expected Results:
 *  1. Recover and halt services are called successively. Afterwards recover and unhold are called once.
 */
TEST_F(AdapterStoTest, testEnableDisableDuringHaltService)
{
  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  // define function for recover-invoke action
  std::function<bool()> sto_false_during_recover_action = [this, &adapter_sto]() {
    this->triggerClearEvent(RECOVER_SRV_CALLED_EVENT);
    adapter_sto.updateSto(false);
    return true;
  };

  std::function<bool()> enable_during_halt_action = [this, &adapter_sto]() {
    this->triggerClearEvent(HALT_SRV_CALLED_EVENT);
    adapter_sto.updateSto(true);
    adapter_sto.updateSto(false); // Important flip!
    return true;
  };

  /**********
   * Step 1 *
   **********/

  {
    InSequence dummy;

    EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _))
        .Times(1)
        .WillOnce(InvokeWithoutArgs(sto_false_during_recover_action));

    EXPECT_CALL(mock_factory_, call_named(HALT_SERVICE, _))
        .Times(1)
        .WillOnce(InvokeWithoutArgs(enable_during_halt_action));
  }

  adapter_sto.updateSto(true);

  BARRIER({RECOVER_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});
}

/**
 * @brief Test enabling with failing recover service and retry (stop plus enable).
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     let recover service fail repeatedly, let unhold service return success
 *  2. Call updateSto(false)),
 *     let hold and halt service return success
 *  3. Call updateSto(true)) repeatedly,
 *     recover and unhold service return success
 *
 * Expected Results:
 *  1. Recover service is called at least once
 *  2. Halt service is called
 *  2. Recover and unhold services are called successively
 */
TEST_F(AdapterStoTest, testRecoverFailPlusRetry)
{
  /**********
   * Step 1 *
   **********/
  EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _))
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(RECOVER_SRV_CALLED_EVENT), Return(false)))
      .WillRepeatedly(Return(false));

  // unhold is optional here
  EXPECT_CALL(mock_factory_, call_named(UNHOLD_SERVICE, _))
      .WillRepeatedly(Return(true));

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  adapter_sto.updateSto(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;

    // hold and is_executing and is optional here
    EXPECT_CALL(mock_factory_, call_named(HOLD_SERVICE, _))
        .WillRepeatedly(Return(true));

    EXPECT_HALT;
  }

  adapter_sto.updateSto(false);

  BARRIER(HALT_SRV_CALLED_EVENT);

  /**********
   * Step 3 *
   **********/
  {
    InSequence dummy;

    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  std::atomic_bool keep_spamming{true};
  std::thread spam_enable{[&adapter_sto, &keep_spamming]() { while (keep_spamming) { adapter_sto.updateSto(true); } }};

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  keep_spamming = false;
  spam_enable.join();
}

/**
 * @brief Test if a stop is possible after unhold failed.
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     let recover service return success and unhold service fail repeatedly
 *  2. Call updateSto(false)),
 *     let hold and halt services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively, the latter one at least once
 *  2. Hold and halt services are called successively.
 */
TEST_F(AdapterStoTest, testUnholdFail)
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

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;

    EXPECT_HOLD;
    EXPECT_HALT;
  }

  adapter_sto.updateSto(false);

  BARRIER2({HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});
}

/**
 * @brief Test if stop is continued properly despite failing hold service
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     let recover and unhold services return success
 *  2. Call updateSto(false)),
 *     let hold service fail, let is_executing service return "not executing",
 *     let halt service fail (for full coverage)
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold, is_executing and halt services are called successively
 */
TEST_F(AdapterStoTest, testHoldFail)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(mock_factory_, call_named(HOLD_SERVICE, _))
        .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(HOLD_SRV_CALLED_EVENT), Return(false)))
        .WillRepeatedly(Return(false));

    EXPECT_CALL(mock_factory_, call_named(HALT_SERVICE, _))
        .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(HALT_SRV_CALLED_EVENT), Return(false)))
        .WillRepeatedly(Return(false));
  }

  adapter_sto.updateSto(false);

  BARRIER2({HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});
}

/**
 * @brief Test hold immediately when sto changes to false during unhold.
 *
 * @note This test exists mainly for full function coverage.
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     call updateSto(false)) during unhold service call and return success,
 *     let hold and halt services return success
 *
 * Expected Results:
 *  1. Recover, unhold, hold and halt services are called successively
 */
TEST_F(AdapterStoTest, testHoldImmediatelyAfterUnhold)
{
  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  // define function for unhold-invoke action
  std::function<bool()> unhold_action = [this, &adapter_sto]() {
    this->triggerClearEvent(UNHOLD_SRV_CALLED_EVENT);
    adapter_sto.updateSto(false);
    return true;
  };

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_RECOVER;

    EXPECT_CALL(mock_factory_, call_named(UNHOLD_SERVICE, _))
        .WillOnce(InvokeWithoutArgs(unhold_action));

    EXPECT_HOLD;
    EXPECT_HALT;
  }

  adapter_sto.updateSto(true);

  barricade({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT, HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});
}

/**
 * @brief Test stopping the state machine in state Enabling.
 *
 * @note This test exists mainly for full function coverage.
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     call stopStateMachine() during recover service call and return success,
 *     let unhold service return success
 *
 * Expected Results:
 *  1. Recover service is called
 */
TEST_F(AdapterStoTest, testExitInStateEnabling)
{
  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  // define function for recover-invoke action
  std::function<bool()> recover_action = [this, &adapter_sto]() {
    adapter_sto.stopStateMachine();
    this->triggerClearEvent(RECOVER_SRV_CALLED_EVENT);
    return true;
  };

  {
    InSequence dummy;

    EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _))
        .WillOnce(InvokeWithoutArgs(recover_action));

    // do not exclude other service calls as stopping the state machine does not prevent further actions
    EXPECT_CALL(mock_factory_, call_named(UNHOLD_SERVICE, _))
        .WillRepeatedly(Return(true));
  }

  adapter_sto.updateSto(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
}

/**
 * @brief Test stopping the state machine in state StopRequestedDuringRecover.
 *
 * @note This test exists mainly for full function coverage.
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     call updateSto(false)) and stopStateMachine() during unhold service call and return success,
 *     let hold and halt services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 */
TEST_F(AdapterStoTest, testExitInStateStopRequestedDuringEnable)
{
  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  // define function for unhold-invoke action
  std::function<bool()> unhold_action = [this, &adapter_sto]() {
    adapter_sto.updateSto(false);
    adapter_sto.stopStateMachine();
    this->triggerClearEvent(UNHOLD_SRV_CALLED_EVENT);
    return true;
  };

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_RECOVER;
    EXPECT_CALL(mock_factory_, call_named(UNHOLD_SERVICE, _))
        .WillOnce(InvokeWithoutArgs(unhold_action));

    // do not exclude other service calls as stopping the state machine does not prevent further actions
    EXPECT_CALL(mock_factory_, call_named(HOLD_SERVICE, _))
        .WillRepeatedly(Return(true));

    EXPECT_CALL(mock_factory_, call_named(HALT_SERVICE, _))
        .WillRepeatedly(Return(true));
  }

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});
}

/**
 * @brief Test stopping the state machine in state StopRequestedDuringRecover.
 *
 * @note This test exists mainly for full function coverage.
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     call updateSto(false)) and stopStateMachine() during unhold service call and return success,
 *     let hold and halt services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 */
TEST_F(AdapterStoTest, testExitDuringPendingHaltCall)
{
  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1
                                                                       , std::placeholders::_2)};

  // define function for recover-invoke action
  std::function<bool()> sto_false_during_recover_action = [this, &adapter_sto]() {
    this->triggerClearEvent(RECOVER_SRV_CALLED_EVENT);
    adapter_sto.updateSto(false);
    return true;
  };

  std::function<bool()> false_recover_action = []() {
    return false;
  };

  std::function<bool()> enable_during_halt_action = [this, &adapter_sto]() {
    adapter_sto.updateSto(true);
    adapter_sto.stopStateMachine();
    this->triggerClearEvent(HALT_SRV_CALLED_EVENT);
    return true;
  };

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _))
        .WillOnce(InvokeWithoutArgs(sto_false_during_recover_action));

    EXPECT_CALL(mock_factory_, call_named(HALT_SERVICE, _))
        .WillOnce(InvokeWithoutArgs(enable_during_halt_action));

    EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _))
        .WillOnce(InvokeWithoutArgs(false_recover_action));
  }

  adapter_sto.updateSto(true);

  BARRIER({RECOVER_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});
}

} // namespace prbt_hardware_support_tests

int main(int argc, char **argv)
{
  // for (limited) ros::Time functionality, no ROS communication
  ros::Time::init();

  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}