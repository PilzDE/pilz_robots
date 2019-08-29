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

#include <pilz_testutils/async_test.h>
#include <prbt_hardware_support/FrameSpeeds.h>
#include <prbt_hardware_support/ros_test_helper.h>
#include <prbt_hardware_support/speed_observer.h>
#include <prbt_hardware_support/wait_for_service.h>

namespace speed_observer_test
{
using ::testing::_;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::SetArgReferee;

using namespace prbt_hardware_support;

static const std::string BARRIER_SLOW{ "BARRIER_SLOW" };
static const std::string BARRIER_FAST{ "BARRIER_FAST" };

static const std::string FRAME_SPEEDS_TOPIC_NAME{ "/frame_speeds" };
static const std::string STOP_TOPIC_NAME{ "/safe_torque_off" };
static const std::string TEST_BASE_FRAME{ "test_base" };
static const std::string TEST_FRAME_A{ "a" };
static const std::string TEST_FRAME_B{ "b" };
static const double TEST_FREQUENCY{ 20 };
static const double SQRT_2_HALF{ 1 / sqrt(2) };
static const double PI_2{ 2 * M_PI };

class SpeedObserverIntegarionTest : public testing::Test, public testing::AsyncTest
{
public:
  void SetUp() override;
  void TearDown() override;

  MOCK_METHOD1(frame_speeds_cb_mock, void(FrameSpeeds msg));
  MOCK_METHOD2(stop_cb_mock, bool(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res));
  void publishTfAtSpeed(double v);
  void stopTfPublisher();

protected:
  ros::Subscriber speed_subscriber_;
  ros::ServiceServer stop_subscriber_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_{ "~" };
  std::vector<std::string> additional_frames_;
  bool tf_publisher_running{ false };
};

void SpeedObserverIntegarionTest::SetUp()
{
  speed_subscriber_ =
      nh_.subscribe<FrameSpeeds>(FRAME_SPEEDS_TOPIC_NAME, 1, &SpeedObserverIntegarionTest::frame_speeds_cb_mock, this);
  stop_subscriber_ = nh_.advertiseService(STOP_TOPIC_NAME, &SpeedObserverIntegarionTest::stop_cb_mock, this);

  waitForService(STOP_TOPIC_NAME);
}

void SpeedObserverIntegarionTest::TearDown()
{
}

void SpeedObserverIntegarionTest::publishTfAtSpeed(double v)
{
  static tf2_ros::TransformBroadcaster br;
  ros::Rate r = ros::Rate(TEST_FREQUENCY * 3);  // publishing definitely faster then observing
  ros::Time start = ros::Time::now();
  double t = 0;
  tf_publisher_running = true;
  while (tf_publisher_running)
  {
    ros::Time current = ros::Time::now();
    t = (current - start).toSec();
    // a has no speed
    geometry_msgs::TransformStamped tfsa;
    tfsa.header.stamp = current;
    tfsa.header.frame_id = TEST_BASE_FRAME;
    tfsa.child_frame_id = TEST_FRAME_A;
    tfsa.transform.translation.x = 99.0;
    tfsa.transform.translation.y = 99.0;
    tfsa.transform.translation.z = 99.0;
    tfsa.transform.rotation.w = 1;
    // b has speed v
    geometry_msgs::TransformStamped tfsb;
    tfsb.header.stamp = current;
    tfsb.header.frame_id = TEST_BASE_FRAME;
    tfsb.child_frame_id = TEST_FRAME_B;
    // rotation in a tilted circle to cover all axis
    tfsb.transform.translation.x = v * cos(t);
    tfsb.transform.translation.y = v * SQRT_2_HALF * -sin(t);
    tfsb.transform.translation.z = v * SQRT_2_HALF * sin(t);
    tfsb.transform.rotation.w = 1;
    br.sendTransform(tfsa);
    br.sendTransform(tfsb);

    if (tf_publisher_running)  // ending faster
      r.sleep();
  }
}

void SpeedObserverIntegarionTest::stopTfPublisher()
{
  tf_publisher_running = false;
}

MATCHER_P(ContainsName, name,
          "Message " + std::string(negation ? "does not contain" : "contains") + " name: " + name + ".")
{
  return arg.name.end() != std::find(arg.name.begin(), arg.name.end(), name);
}

using ::testing::PrintToString;
MATCHER_P2(SpeedAtIGe, i, x,
           "Speed at index " + PrintToString(i) + std::string(negation ? "is not" : "is") + " greater or equal to" +
               PrintToString(x) + ".")
{
  return arg.speed[i] >= x;
}

MATCHER_P2(SpeedAtILe, i, x,
           "Speed at index " + PrintToString(i) + std::string(negation ? "is not" : "is") + " less or equal to" +
               PrintToString(x) + ".")
{
  return arg.speed[i] <= x;
}

/**
 * @tests{The correct observation of frames}
 *
 * Test Sequence:
 *    0. Starting up
 *    1. Publishing tf movements that *not* have a too high speed
 *
 * Expected Results:
 *    0. -
 *    1. Correct values are publshed on the speed topic.
 *       *No* stop is published
 */
TEST_F(SpeedObserverIntegarionTest, testStartupAndTopic)
{
  using ::testing::AllOf;
  using ::testing::AtLeast;

  /**********
   * Step 0 *
   **********/
  ROS_DEBUG_STREAM("Step 0");
  std::string reference_frame{ TEST_BASE_FRAME };
  std::vector<std::string> frames_to_observe{ TEST_FRAME_A, TEST_FRAME_B };
  prbt_hardware_support::SpeedObserver observer(nh_, reference_frame, frames_to_observe);
  std::thread observer_thread = std::thread(&SpeedObserver::startObserving, &observer, TEST_FREQUENCY);
  ROS_DEBUG_STREAM("thread started");

  /**********
   * Step 1 *
   **********/
  ROS_DEBUG_STREAM("Step 1");
  ::testing::Sequence s_speeds;
  EXPECT_CALL(*this, frame_speeds_cb_mock(AllOf(ContainsName(TEST_FRAME_A), ContainsName(TEST_FRAME_B),
                                                SpeedAtILe((unsigned long)0, .1), SpeedAtILe((unsigned long)1, .26),
                                                SpeedAtIGe((unsigned long)0, 0), SpeedAtIGe((unsigned long)1, 0))))
      .Times(AtLeast(2))
      .InSequence(s_speeds);
  EXPECT_CALL(*this, frame_speeds_cb_mock(AllOf(ContainsName(TEST_FRAME_A), ContainsName(TEST_FRAME_B),
                                                SpeedAtILe((unsigned long)0, .1), SpeedAtILe((unsigned long)1, .26),
                                                SpeedAtIGe((unsigned long)0, 0), SpeedAtIGe((unsigned long)1, 0))))
      .Times(AtLeast(1))
      .InSequence(s_speeds)
      .WillRepeatedly(ACTION_OPEN_BARRIER_VOID(BARRIER_SLOW));

  EXPECT_CALL(*this, stop_cb_mock(_, _)).Times(0);

  std::thread pubisher_thread_slow = std::thread(&SpeedObserverIntegarionTest::publishTfAtSpeed, this, 0.24);
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
 * @tests{The correct handling of too high speeds}
 *
 * Test Sequence:
 *    0. Starting up
 *    1. Publishing tf movements that have a too high speed
 *
 * Expected Results:
 *    0. -
 *    1. Correct values are publshed on the speed topic.
 *       A stop is published
 */
TEST_F(SpeedObserverIntegarionTest, testTooHighSpeed)
{
  using ::testing::AllOf;
  using ::testing::AtLeast;
  using ::testing::AtMost;

  /**********
   * Step 0 *
   **********/
  ROS_DEBUG_STREAM("Step 0");
  std::string reference_frame{ TEST_BASE_FRAME };
  std::vector<std::string> frames_to_observe{ TEST_FRAME_A, TEST_FRAME_B };
  prbt_hardware_support::SpeedObserver observer(nh_, reference_frame, frames_to_observe);
  std::thread observer_thread = std::thread(&SpeedObserver::startObserving, &observer, TEST_FREQUENCY);
  ROS_DEBUG_STREAM("thread started");

  /**********
   * Step 1 *
   **********/
  ROS_DEBUG_STREAM("Step 1");
  ::testing::Sequence s_speeds;
  // we can once have a slow speed, when the publisher starts:
  EXPECT_CALL(*this, frame_speeds_cb_mock(AllOf(ContainsName(TEST_FRAME_A), ContainsName(TEST_FRAME_B),
                                                SpeedAtILe((unsigned long)0, .1), SpeedAtILe((unsigned long)1, .32),
                                                SpeedAtIGe((unsigned long)0, 0), SpeedAtIGe((unsigned long)1, 0))))
      .Times(AtMost(1))
      .InSequence(s_speeds);
  EXPECT_CALL(*this, frame_speeds_cb_mock(AllOf(ContainsName(TEST_FRAME_A), ContainsName(TEST_FRAME_B),
                                                SpeedAtILe((unsigned long)0, .1), SpeedAtILe((unsigned long)1, .32),
                                                SpeedAtIGe((unsigned long)0, 0), SpeedAtIGe((unsigned long)1, .28))))
      .Times(AtLeast(2))
      .InSequence(s_speeds);
  EXPECT_CALL(*this, frame_speeds_cb_mock(AllOf(ContainsName(TEST_FRAME_A), ContainsName(TEST_FRAME_B),
                                                SpeedAtILe((unsigned long)0, .1), SpeedAtILe((unsigned long)1, .32),
                                                SpeedAtIGe((unsigned long)0, 0), SpeedAtIGe((unsigned long)1, .28))))
      .Times(AtLeast(1))
      .InSequence(s_speeds)
      .WillRepeatedly(ACTION_OPEN_BARRIER_VOID(BARRIER_FAST));

  std_srvs::SetBool::Response sto_srv_resp;
  sto_srv_resp.success = true;

  EXPECT_CALL(*this, stop_cb_mock(_, _))
      .Times(AtLeast(1))
      .WillOnce(DoAll(SetArgReferee<1>(sto_srv_resp), Return(true)));

  std::thread pubisher_thread_fast = std::thread(&SpeedObserverIntegarionTest::publishTfAtSpeed, this, 0.3);
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
 * @tests{The correct handling of too high speeds}
 *
 * Test Sequence:
 *    0. Starting up
 *    1. Publishing tf movements only for a short while
 *
 * Expected Results:
 *    0. -
 *    1. observe method just finishes (without throw)
 */
TEST_F(SpeedObserverIntegarionTest, testTimeout)
{
  /**********
   * Step 0 *
   **********/
  ROS_DEBUG_STREAM("Step 0");
  std::string reference_frame{ TEST_BASE_FRAME };
  std::vector<std::string> frames_to_observe{ TEST_FRAME_A, TEST_FRAME_B };
  prbt_hardware_support::SpeedObserver observer(nh_, reference_frame, frames_to_observe);

  /**********
   * Step 1 *
   **********/
  ROS_DEBUG_STREAM("Step 1");
  EXPECT_NO_THROW(observer.startObserving(TEST_FREQUENCY));
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