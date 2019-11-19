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

#include <thread>

#include <gmock/gmock.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pilz_utils/wait_for_service.h>
#include <pilz_testutils/async_test.h>
#include <prbt_hardware_support/FrameSpeeds.h>
#include <prbt_hardware_support/SetSpeedLimit.h>
#include <prbt_hardware_support/ros_test_helper.h>
#include <prbt_hardware_support/speed_observer.h>

namespace speed_observer_test
{
using ::testing::_;
using ::testing::AllOf;
using ::testing::AtLeast;
using ::testing::AtMost;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::SetArgReferee;

using namespace prbt_hardware_support;

static const std::string BARRIER_SLOW{ "BARRIER_SLOW" };
static const std::string BARRIER_FAST{ "BARRIER_FAST" };
static const std::string BARRIER_LIMIT{ "BARRIER_LIMIT" };
static const std::string BARRIER_LIMIT_LOW{ "BARRIER_LIMIT_LOW" };

static const std::string FRAME_SPEEDS_TOPIC_NAME{ "/frame_speeds" };
static const std::string STOP_TOPIC_NAME{ "/safe_torque_off" };
static const std::string TEST_BASE_FRAME{ "test_base" };
static const std::string TEST_FRAME_A{ "a" };
static const std::string TEST_FRAME_B{ "b" };
static constexpr double TEST_FREQUENCY{ 20 };
// Publishing definitely faster then observing
static constexpr double OBSERVER_FREQUENCY {3.0 * TEST_FREQUENCY};
static const double SQRT_2_HALF{ 1 / sqrt(2) };

using ::testing::AllOf;
using ::testing::AtLeast;
using ::testing::AtMost;

class SpeedObserverUnitTest : public testing::Test, public testing::AsyncTest
{
public:
  void SetUp() override;

  MOCK_METHOD1(frame_speeds_cb_mock, void(const FrameSpeeds::ConstPtr& msg));
  MOCK_METHOD2(stop_cb_mock, bool(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res));
  void publishTfAtSpeed(const double v, const double observer_frequency = OBSERVER_FREQUENCY);
  void stopTfPublisher();

protected:
  ros::Subscriber speed_subscriber_;
  ros::ServiceServer stop_subscriber_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_{ "~" };
  std::vector<std::string> additional_frames_;
  bool tf_publisher_running_{ false };
};

void SpeedObserverUnitTest::SetUp()
{
  speed_subscriber_ =
      nh_.subscribe<FrameSpeeds>(FRAME_SPEEDS_TOPIC_NAME, 1, &SpeedObserverUnitTest::frame_speeds_cb_mock, this);
  stop_subscriber_ = nh_.advertiseService(STOP_TOPIC_NAME, &SpeedObserverUnitTest::stop_cb_mock, this);

  pilz_utils::waitForService(STOP_TOPIC_NAME);
}

void SpeedObserverUnitTest::publishTfAtSpeed(const double speed, const double publish_frequency)
{
  static tf2_ros::TransformBroadcaster br;
  ros::Rate r = ros::Rate(publish_frequency);
  ros::Time start = ros::Time::now();
  double t = 0;
  tf_publisher_running_ = true;
  while (tf_publisher_running_)
  {
    ros::Time current = ros::Time::now();
    t = (current - start).toSec();
    // a has no speed
    geometry_msgs::TransformStamped transform_stamped_a;
    transform_stamped_a.header.stamp = current;
    transform_stamped_a.header.frame_id = TEST_BASE_FRAME;
    transform_stamped_a.child_frame_id = TEST_FRAME_A;
    transform_stamped_a.transform.translation.x = 99.0;
    transform_stamped_a.transform.translation.y = 99.0;
    transform_stamped_a.transform.translation.z = 99.0;
    transform_stamped_a.transform.rotation.w = 1;
    // b has speed v
    geometry_msgs::TransformStamped transform_stamped_b;
    transform_stamped_b.header.stamp = current;
    transform_stamped_b.header.frame_id = TEST_BASE_FRAME;
    transform_stamped_b.child_frame_id = TEST_FRAME_B;
    // rotation in a tilted circle to cover all axis
    transform_stamped_b.transform.translation.x = speed * cos(t);
    transform_stamped_b.transform.translation.y = speed * SQRT_2_HALF * -sin(t);
    transform_stamped_b.transform.translation.z = speed * SQRT_2_HALF * sin(t);
    transform_stamped_b.transform.rotation.w = 1;
    br.sendTransform(transform_stamped_a);
    br.sendTransform(transform_stamped_b);

    if (tf_publisher_running_)  // ending faster
      r.sleep();
  }
}

void SpeedObserverUnitTest::stopTfPublisher()
{
  tf_publisher_running_ = false;
}

using ::testing::PrintToString;
MATCHER_P(StoState, x, "Sto state " + std::string(negation ? "is not" : "is") + ": " + PrintToString(x) + ".")
{
  return arg.data == x;
}

MATCHER_P2(NameAtI, i, name,
           "Name at index " + PrintToString(i) + std::string(negation ? "is not" : "is") + ": " + name + ".")
{
  return static_cast<size_t>(i) < arg->name.size() && arg->name.at(static_cast<size_t>(i)).compare(name) == 0;
}

MATCHER_P2(SpeedAtIGe, i, x,
           "Speed at index " + PrintToString(i) + std::string(negation ? "is not" : "is") + " greater or equal to" +
           PrintToString(x) + ".")
{
  return static_cast<size_t>(i) < arg->name.size() && arg->speed.at(static_cast<size_t>(i)) >= x;
}

MATCHER_P2(SpeedAtILe, i, x,
           "Speed at index " + PrintToString(i) + std::string(negation ? "is not" : "is") + " less or equal to" +
           PrintToString(x) + ".")
{
  return static_cast<size_t>(i) < arg->name.size() && arg->speed.at(static_cast<size_t>(i)) <= x;
}

/**
 * @tests{Monitor_Speed_of_all_tf_frames_until_TCP,
 * Tests the correct observation of frames and handling of slow speeds.
 * }
 *
 * Test Sequence:
 *    1. Publish tf movements that are within the speed limit
 *    2. Start speed observing
 *
 * Expected Results:
 *    1. -
 *    2. Correct values are published on the speed topic.
 *       *No* stop is published
 */
TEST_F(SpeedObserverUnitTest, testStartupAndTopic)
{
  /**********
   * Step 1 *
   **********/
  ROS_DEBUG_STREAM("Step 1");
  std::thread pubisher_thread_slow = std::thread(&SpeedObserverUnitTest::publishTfAtSpeed, this, 0.24, OBSERVER_FREQUENCY);

  /**********
   * Step 2 *
   **********/
  ROS_DEBUG_STREAM("Step 2");
  EXPECT_CALL(*this, frame_speeds_cb_mock(AllOf(NameAtI(0, TEST_FRAME_A), NameAtI(1, TEST_FRAME_B),
                                                SpeedAtILe(0, .1), SpeedAtILe(1, .26),
                                                SpeedAtIGe(0, 0), SpeedAtIGe(1, 0))))
      .WillOnce(Return())
      .WillOnce(Return())
      .WillOnce(ACTION_OPEN_BARRIER_VOID(BARRIER_SLOW))
      .WillRepeatedly(Return());

  EXPECT_CALL(*this, stop_cb_mock(_, _)).Times(0);

  std::string reference_frame{ TEST_BASE_FRAME };
  std::vector<std::string> frames_to_observe{ TEST_FRAME_A, TEST_FRAME_B };
  prbt_hardware_support::SpeedObserver observer(nh_, reference_frame, frames_to_observe);
  std::thread observer_thread = std::thread(&SpeedObserver::startObserving,
                                            &observer,
                                            TEST_FREQUENCY,
                                            DEFAULT_ALLOWED_MISSED_CALCULATIONS);
  ROS_DEBUG_STREAM("thread started");

  BARRIER({ BARRIER_SLOW });
  stopTfPublisher();
  pubisher_thread_slow.join();

  /*************
   * Tear Down *
   *************/
  ROS_DEBUG_STREAM("Tear Down");
  observer.terminateNow();
  observer_thread.join();
}

/**
 * @tests{Monitor_Speed_of_all_tf_frames_until_TCP,
 * Tests the correct handling of too high speeds.
 * }
 * @tests{Stop1_on_violation_of_speed_limit,
 * Tests that Stop 1 is triggered if speed limit is violated.
 * }
 *
 * Test Sequence:
 *    1. Publish tf movements that have a too high speed
 *    2. Start speed observing
 *
 * Expected Results:
 *    1. -
 *    2. Correct values are publshed on the speed topic.
 *       A stop is published
 */
TEST_F(SpeedObserverUnitTest, testTooHighSpeed)
{
  /**********
   * Step 1 *
   **********/
  ROS_DEBUG_STREAM("Step 1");
  std::thread pubisher_thread_fast = std::thread(&SpeedObserverUnitTest::publishTfAtSpeed, this, 0.3, OBSERVER_FREQUENCY);

  /**********
   * Step 2 *
   **********/
  ROS_DEBUG_STREAM("Step 2");
  // we can once have a slow speed, when the observer starts:
  EXPECT_CALL(*this, frame_speeds_cb_mock(AllOf(NameAtI(0, TEST_FRAME_A), NameAtI(1, TEST_FRAME_B),
                                                SpeedAtILe(0, .1), SpeedAtILe(1, .32),
                                                SpeedAtIGe(0, 0), SpeedAtIGe(1, 0))))
      .Times(AtMost(1));
  EXPECT_CALL(*this, frame_speeds_cb_mock(AllOf(NameAtI(0, TEST_FRAME_A), NameAtI(1, TEST_FRAME_B),
                                                SpeedAtILe(0, .1), SpeedAtILe(1, .32),
                                                SpeedAtIGe(0, 0), SpeedAtIGe(1, .28))));

  std_srvs::SetBool::Response sto_srv_resp;
  sto_srv_resp.success = true;

  EXPECT_CALL(*this, stop_cb_mock(StoState(false), _))
      .WillRepeatedly(DoAll(SetArgReferee<1>(sto_srv_resp), ACTION_OPEN_BARRIER(BARRIER_FAST)));

  std::string reference_frame{ TEST_BASE_FRAME };
  std::vector<std::string> frames_to_observe{ TEST_FRAME_A, TEST_FRAME_B };
  prbt_hardware_support::SpeedObserver observer(nh_, reference_frame, frames_to_observe);
  std::thread observer_thread = std::thread(&SpeedObserver::startObserving,
                                            &observer,
                                            TEST_FREQUENCY,
                                            DEFAULT_ALLOWED_MISSED_CALCULATIONS);
  ROS_DEBUG_STREAM("thread started");

  BARRIER({ BARRIER_FAST });
  stopTfPublisher();
  pubisher_thread_fast.join();

  /*************
   * Tear Down *
   *************/
  ROS_DEBUG_STREAM("Tear Down");
  observer.terminateNow();
  observer_thread.join();
}

/**
 * @tests{Monitor_Speed_of_all_tf_frames_until_TCP,
 * Tests the correct handling of changed speed limits.
 * }
 * @tests{Speed_limits_per_operation_mode,
 * Test that speed limits can be changed.
 * }
 * @tests{Stop1_on_violation_of_speed_limit,
 * Tests that Stop 1 is triggered if after changing the speed limit.
 * }
 *
 * Test Sequence:
 *    1. Publish tf movements with speed of 0.3,
 *    1. Start observer, set speed limit to 0.4
 *    2. Reduce speed limit to 0.2
 *
 * Expected Results:
 *    1. -
 *    2. Correct values are published on the speed topic.
 *       *No* stop is called
 *    3. A stop is called
 */
TEST_F(SpeedObserverUnitTest, testSetSpeedLimit)
{
  /**********
   * Step 1 *
   **********/
  ROS_DEBUG_STREAM("Step 1");
  std::thread pubisher_thread_fast = std::thread(&SpeedObserverUnitTest::publishTfAtSpeed, this, 0.3, OBSERVER_FREQUENCY);

  /**********
   * Step 2 *
   **********/
  ROS_DEBUG_STREAM("Step 2");

  // we can once have a slow speed, when the observer starts:
  EXPECT_CALL(*this, frame_speeds_cb_mock(AllOf(NameAtI(0, TEST_FRAME_A), NameAtI(1, TEST_FRAME_B),
                                                SpeedAtILe(0, .1), SpeedAtILe(1, .32),
                                                SpeedAtIGe(0, 0), SpeedAtIGe(1, 0))))
      .Times(AtMost(1));
  EXPECT_CALL(*this, frame_speeds_cb_mock(AllOf(NameAtI(0, TEST_FRAME_A), NameAtI(1, TEST_FRAME_B),
                                                SpeedAtILe(0, .1), SpeedAtILe(1, .32),
                                                SpeedAtIGe(0, 0), SpeedAtIGe(1, .28))))
      .WillOnce(Return())
      .WillOnce(Return())
      .WillOnce(ACTION_OPEN_BARRIER_VOID(BARRIER_LIMIT))
      .WillRepeatedly(Return());
  EXPECT_CALL(*this, stop_cb_mock(_, _)).Times(0);

  std::string reference_frame{ TEST_BASE_FRAME };
  std::vector<std::string> frames_to_observe{ TEST_FRAME_A, TEST_FRAME_B };
  prbt_hardware_support::SpeedObserver observer(nh_, reference_frame, frames_to_observe);
  SetSpeedLimitRequest req = SetSpeedLimitRequest();
  SetSpeedLimitResponse res = SetSpeedLimitResponse();
  req.speed_limit = .4;
  observer.setSpeedLimitCb(req, res);
  std::thread observer_thread = std::thread(&SpeedObserver::startObserving,
                                            &observer,
                                            TEST_FREQUENCY,
                                            DEFAULT_ALLOWED_MISSED_CALCULATIONS);
  ROS_DEBUG_STREAM("thread started");

  BARRIER({ BARRIER_LIMIT });

  /**********
   * Step 3 *
   **********/
  ROS_DEBUG_STREAM("Step 3");
  std_srvs::SetBool::Response sto_srv_resp;
  sto_srv_resp.success = true;
  EXPECT_CALL(*this, stop_cb_mock(StoState(false), _))
      .WillRepeatedly(DoAll(SetArgReferee<1>(sto_srv_resp), ACTION_OPEN_BARRIER(BARRIER_LIMIT_LOW)));

  req.speed_limit = .2;
  observer.setSpeedLimitCb(req, res);

  BARRIER({ BARRIER_LIMIT_LOW });

  /*************
   * Tear Down *
   *************/
  ROS_DEBUG_STREAM("Tear Down");
  stopTfPublisher();
  pubisher_thread_fast.join();
  observer.terminateNow();
  observer_thread.join();
}

/**
 * @tests{Monitor_Speed_of_all_tf_frames_until_TCP,
 * Tests the correct handling of no tf.
 * }
 * @tests{Monitor_Speed_of_user_defined_tf_frames,
 * Tests the correct handling of no tf.
 * }
 *
 * Test Sequence:
 *    1. Start observer
 *
 * Expected Results:
 *    1. observe method throws exception
 */
TEST_F(SpeedObserverUnitTest, testTimeout)
{
  /**********
   * Step 1 *
   **********/
  ROS_DEBUG_STREAM("Step 1");
  std::string reference_frame{ TEST_BASE_FRAME };
  std::vector<std::string> frames_to_observe{ TEST_FRAME_A };
  prbt_hardware_support::SpeedObserver observer(nh_, reference_frame, frames_to_observe);

  EXPECT_THROW(observer.startObserving(TEST_FREQUENCY), std::runtime_error);
}

/**
 * Test case in which STO service call fails (needed for line coverage).
 *
 * Test Sequence:
 *    1. Publish tf movements that have a too high speed
 *    2. Start speed observing
 *
 * Expected Results:
 *    1. -
 *    2. Sto service is called
 */
TEST_F(SpeedObserverUnitTest, testFailingStoServiceCase)
{
  // ++++++++++++++++++++++++
  ROS_DEBUG_STREAM("Step 1");
  // ++++++++++++++++++++++++
  std::thread pubisher_thread_fast = std::thread(&SpeedObserverUnitTest::publishTfAtSpeed, this, 0.3, OBSERVER_FREQUENCY);

  // ++++++++++++++++++++++++
  ROS_DEBUG_STREAM("Step 2");
  // ++++++++++++++++++++++++
  EXPECT_CALL(*this, stop_cb_mock(StoState(false), _))
      .Times(1)
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(BARRIER_FAST), Return(false)));

  std::string reference_frame{ TEST_BASE_FRAME };
  std::vector<std::string> frames_to_observe{ TEST_FRAME_A, TEST_FRAME_B };
  prbt_hardware_support::SpeedObserver observer(nh_, reference_frame, frames_to_observe);
  std::thread observer_thread = std::thread(&SpeedObserver::startObserving,
                                            &observer,
                                            TEST_FREQUENCY,
                                            DEFAULT_ALLOWED_MISSED_CALCULATIONS);
  ROS_DEBUG_STREAM("thread started");

  BARRIER({ BARRIER_FAST });

  // ++++++++++++++++++++++++
  ROS_DEBUG_STREAM("Tear Down");
  // ++++++++++++++++++++++++
  stopTfPublisher();
  pubisher_thread_fast.join();
  observer.terminateNow();
  observer_thread.join();
}

/**
 * Tests that Stop1 is triggered in case the frame speed calculation fails
 * too often.
 *
 * Test Sequence:
 *    1. Publish tf movements
 *    2. Start speed observing
 *
 * Expected Results:
 *    1. -
 *    2. Stop is triggered
 */
TEST_F(SpeedObserverUnitTest, testSlowTfPublishing)
{
  // ++++++++++++++++++++++++
  ROS_DEBUG_STREAM("Step 1");
  // ++++++++++++++++++++++++
  std::thread pubisher_thread_fast = std::thread(&SpeedObserverUnitTest::publishTfAtSpeed, this, 0.24, 0.1*TEST_FREQUENCY);

  // ++++++++++++++++++++++++
  ROS_DEBUG_STREAM("Step 2");
  // ++++++++++++++++++++++++
  EXPECT_CALL(*this, stop_cb_mock(StoState(false), _))
      .Times(AtLeast(1))
      .WillRepeatedly(DoAll(ACTION_OPEN_BARRIER_VOID(BARRIER_FAST), Return(true)));

  std::string reference_frame{ TEST_BASE_FRAME };
  std::vector<std::string> frames_to_observe{ TEST_FRAME_A, TEST_FRAME_B };
  prbt_hardware_support::SpeedObserver observer(nh_, reference_frame, frames_to_observe);
  std::thread observer_thread = std::thread(&SpeedObserver::startObserving,
                                            &observer,
                                            TEST_FREQUENCY,
                                            DEFAULT_ALLOWED_MISSED_CALCULATIONS);
  ROS_DEBUG_STREAM("thread started");

  BARRIER({ BARRIER_FAST });

  // ++++++++++++++++++++++++
  ROS_DEBUG_STREAM("Tear Down");
  // ++++++++++++++++++++++++
  stopTfPublisher();
  pubisher_thread_fast.join();
  observer.terminateNow();
  observer_thread.join();
}

}  // namespace speed_observer_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unittest_speed_observer");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 2 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
