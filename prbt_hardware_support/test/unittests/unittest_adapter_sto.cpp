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

#define EXPECT_IS_EXECUTING                                  \
  EXPECT_CALL(mock_factory_, call_named(IS_EXECUTING_SERVICE, _)) \
      .WillOnce(Invoke(isExecutingInvokeAction(true)))

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

std::function<bool(const std::string &name, std_srvs::Trigger &srv)> isExecutingInvokeAction(bool result)
{
  return [result](const std::string &name, std_srvs::Trigger &srv){ srv.response.success = result; return true; };
}

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
  AdapterSto(std::function<ClientMock(std::string)> create_service_client)
    : AdapterStoTemplated<ClientMock>(create_service_client)
  {
  }

  FRIEND_TEST(AdapterStoTest, testEnable);
  FRIEND_TEST(AdapterStoTest, testEnableStopEnable);
  FRIEND_TEST(AdapterStoTest, testSpamEnablePlusStop);
  FRIEND_TEST(AdapterStoTest, testSpamStoActivePlusEnable);
  FRIEND_TEST(AdapterStoTest, testSkippingHoldPlusEnable);
  FRIEND_TEST(AdapterStoTest, testRecoverFailPlusRetry);
  FRIEND_TEST(AdapterStoTest, testUnholdFail);
  FRIEND_TEST(AdapterStoTest, testHoldFail);
  FRIEND_TEST(AdapterStoTest, testHaltFail);
  FRIEND_TEST(AdapterStoTest, testIsExecutingFail);
};

const std::string RECOVER_SERVICE{AdapterSto::RECOVER_SERVICE};
const std::string UNHOLD_SERVICE{AdapterSto::UNHOLD_SERVICE};
const std::string HOLD_SERVICE{AdapterSto::HOLD_SERVICE};
const std::string HALT_SERVICE{AdapterSto::HALT_SERVICE};
const std::string IS_EXECUTING_SERVICE{AdapterSto::IS_EXECUTING_SERVICE};

/**
 * @brief Test D0 destructor
 *
 * Increases function coverage
 */
TEST_F(AdapterStoTest, testD0estructor)
{
  typedef prbt_hardware_support::AdapterStoTemplated<ClientMock> AdapterSto;
  std::shared_ptr<AdapterSto> adapter_sto{new AdapterSto(std::bind(&MockFactory::create, &mock_factory_,
                                                                    std::placeholders::_1))};
}

/**
 * @brief Test enabling
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     let recover service return success
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

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};

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
 *     let hold, is_executing and halt services return success
 *  3. Call updateSto(true)),
 *     let recover service return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold, is_executing and halt services are called successively
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

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_IS_EXECUTING;
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

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});
}

/**
 * @brief Test spaming enable plus subsequent stop
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)) repeatedly,
 *     let recover and unhold services return success
 *  2. Call updateSto(false)),
 *     let hold, is_executing and halt services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold, is_executing and halt services are called successively
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

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};

  std::atomic_bool keep_spamming{true};
  std::thread spam_enable{[&adapter_sto, &keep_spamming]() { while (keep_spamming) { adapter_sto.updateSto(true); } }};

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  keep_spamming = false;
  spam_enable.join();

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_IS_EXECUTING;
    EXPECT_HALT;
  }

  adapter_sto.updateSto(false);

  BARRIER2({HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});
}

/**
 * @brief Test spamming STO=true plus subsequent enable
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     let recover and unhold services return success
 *  2. Call updateSto(false)) repeatedly,
 *     let hold, is_executing and halt services return success
 *  3. Call updateSto(true)),
 *     let recover and unhold services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold, is_exeuting and halt services are called successively
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

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_IS_EXECUTING;
    EXPECT_HALT;
  }

  std::atomic_bool keep_spamming{true};
  std::thread spam_enable{[&adapter_sto, &keep_spamming]() { while (keep_spamming) { adapter_sto.updateSto(false); } }};

  BARRIER2({HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});

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

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});
}

/**
 * @brief Test skipping hold when sto changes to false during recover. Test also a following enabling.
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     call updateSto(false)) during recover service call and return success,
 *     let hold, is_executing and halt services return success, is_exeuting response is "not executing"
 *  2. Call updateSto(true)),
 *     let recover and unhold services return success
 *
 * Expected Results:
 *  1. Recover and halt services are called successively
 *  2. Recover and unhold services are called successively
 */
TEST_F(AdapterStoTest, testSkippingHoldPlusEnable)
{
  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};

  // define function for recover-invoke action
  std::function<bool()> recover_action = [this, &adapter_sto]() {
    this->triggerClearEvent(RECOVER_SRV_CALLED_EVENT);
    adapter_sto.updateSto(false);
    return true;
  };

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _))
        .WillOnce(InvokeWithoutArgs(recover_action));

    // hold is optional here
    EXPECT_CALL(mock_factory_, call_named(HOLD_SERVICE, _))
        .WillRepeatedly(Return(true));

    EXPECT_CALL(mock_factory_, call_named(IS_EXECUTING_SERVICE, _))
        .WillRepeatedly(Invoke(isExecutingInvokeAction(false)));

    EXPECT_HALT;
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

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});
}

/**
 * @brief Test enabling with failing recover service and retry (stop plus enable).
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     let recover service fail repeatedly
 *  2. Call updateSto(false)) and updateSto(true)),
 *     let hold, is_executing, halt and recover service return success, is_executing response is "not executing"
 *
 * Expected Results:
 *  1. Recover service is called at least once
 *  2. Halt, recover and unhold services are called successively.
 */
TEST_F(AdapterStoTest, testRecoverFailPlusRetry)
{
  /**********
   * Step 1 *
   **********/
  EXPECT_CALL(mock_factory_, call_named(RECOVER_SERVICE, _))
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(RECOVER_SRV_CALLED_EVENT), Return(false)))
      .WillRepeatedly(Return(false));

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};

  adapter_sto.updateSto(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;

    // hold is optional here
    EXPECT_CALL(mock_factory_, call_named(HOLD_SERVICE, _))
        .WillRepeatedly(Return(true));

    EXPECT_CALL(mock_factory_, call_named(IS_EXECUTING_SERVICE, _))
        .WillRepeatedly(Invoke(isExecutingInvokeAction(false)));

    EXPECT_HALT;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  adapter_sto.updateSto(false);
  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});
}

/**
 * @brief Test if a stop is possible after unhold failed.
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     let recover service return success and unhold service fail repeatedly
 *  2. Call updateSto(false)),
 *     let hold, is_executing and halt services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively, the latter one at least once
 *  2. Hold, is_executing and halt services are called successively.
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

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;

    EXPECT_HOLD;
    EXPECT_IS_EXECUTING;
    EXPECT_HALT;
  }

  adapter_sto.updateSto(false);

  BARRIER2({HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});
}

/**
 * @brief Test if stop is continued with halt despite failing hold service
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     let recover and unhold services return success
 *  2. Call updateSto(false)),
 *     let is_executing service return success, let hold service fail (halt service fail too for full coverage)
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

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};

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

    EXPECT_CALL(mock_factory_, call_named(IS_EXECUTING_SERVICE, _))
        .WillRepeatedly(Invoke(isExecutingInvokeAction(false)));

    EXPECT_CALL(mock_factory_, call_named(HALT_SERVICE, _))
        .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(HALT_SRV_CALLED_EVENT), Return(false)))
        .WillRepeatedly(Return(false));
  }

  adapter_sto.updateSto(false);

  BARRIER2({HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});
}

/**
 * @brief Test if stop is continued with halt despite failing is_executing service
 *
 * Test Sequence:
 *  1. Run the sto adapter and call updateSto(true)),
 *     let recover and unhold services return success
 *  2. Call updateSto(false)),
 *     let hald and halt service return success, let is_executing service fail
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold, is_executing and halt services are called successively
 */
TEST_F(AdapterStoTest, testIsExecutingFail)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  AdapterSto adapter_sto{std::bind(&MockFactory::create, &mock_factory_, std::placeholders::_1)};

  adapter_sto.updateSto(true);

  BARRIER2({RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT});

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_CALL(mock_factory_, call_named(IS_EXECUTING_SERVICE, _))
        .WillOnce(Return(false))
        .WillRepeatedly(Return(false));
    EXPECT_HALT;
  }

  adapter_sto.updateSto(false);

  BARRIER2({HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT});
}

} // namespace prbt_hardware_support_tests

int main(int argc, char **argv)
{
  // for (limited) ros::Time functionality, no ROS communication
  ros::Time::init();

  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}