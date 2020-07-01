/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
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
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gmock/gmock.h>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <pilz_testutils/async_test.h>
#include <pilz_testutils/joint_state_publisher_mock.h>

#include <pilz_utils/get_param.h>
#include <pilz_utils/wait_for_message.h>

namespace std_msgs
{
void PrintTo(const std_msgs::Float64ConstPtr& msg, std::ostream* os)
{
  *os << msg->data;
}
}  // namespace std_msgs

namespace joint_states_speed_observer_test
{
static const std::string MAX_FRAME_SPEED_TOPIC_NAME{ "max_frame_speed" };
static const std::string BARRIER_MAX_FRAME_SPEED{ "BARRIER_MAX_FRAME_SPEED" };

static constexpr unsigned int MAX_FRAME_SPEED_QUEUE_SIZE{ 1 };
static constexpr int BARRIER_MAX_FRAME_SPEED_TIMEOUT_MS{ 2000 };

static constexpr double RIGHT_ARM_RADIUS{ 0.25 };
static constexpr double FRAME_SPEED_COMPUTATION_TOLERANCE{ 0.01 };

class MaxFrameSpeedSubscriberMock
{
public:
  MaxFrameSpeedSubscriberMock()
  {
    sub_ = nh_.subscribe<std_msgs::Float64>(MAX_FRAME_SPEED_TOPIC_NAME, MAX_FRAME_SPEED_QUEUE_SIZE,
                                            &MaxFrameSpeedSubscriberMock::cb_mock, this);
  }

  void waitForObserver()
  {
    pilz_utils::waitForMessage<std_msgs::Float64>(MAX_FRAME_SPEED_TOPIC_NAME);
  }

  MOCK_METHOD1(cb_mock, void(const std_msgs::Float64ConstPtr&));

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
};

using testing::AtLeast;
using testing::PrintToString;
using testing::Return;

MATCHER_P(MaxFrameSpeedNear, v,
          "Max frame speed " + std::string(negation ? "is not" : "is") + " near " + PrintToString(v) + ".")
{
  return std::abs(arg->data - v) < FRAME_SPEED_COMPUTATION_TOLERANCE;
}

MATCHER_P(MaxFrameSpeedLe, v,
          "Max frame speed " + std::string(negation ? "is not" : "is") + " less or equal to " + PrintToString(v) + ".")
{
  return std::abs(arg->data) < (std::abs(v) + FRAME_SPEED_COMPUTATION_TOLERANCE);
}

class JointStatesSpeedObserverTest : public testing::Test, public testing::AsyncTest
{
public:
  JointStatesSpeedObserverTest();
  ~JointStatesSpeedObserverTest() override;

protected:
  testing::StrictMock<MaxFrameSpeedSubscriberMock> subscriber_mock_;
  pilz_testutils::JointStatePublisherMock joint_state_pub_;
};

JointStatesSpeedObserverTest::JointStatesSpeedObserverTest()
{
  EXPECT_CALL(subscriber_mock_, cb_mock(MaxFrameSpeedNear(0.0))).Times(AtLeast(0));
  joint_state_pub_.startPublishingAsync();
  subscriber_mock_.waitForObserver();
}

JointStatesSpeedObserverTest::~JointStatesSpeedObserverTest()
{
  ON_CALL(subscriber_mock_, cb_mock(MaxFrameSpeedNear(0.0)))
      .WillByDefault(ACTION_OPEN_BARRIER_VOID(BARRIER_MAX_FRAME_SPEED));
  joint_state_pub_.goHome();
  BARRIER(BARRIER_MAX_FRAME_SPEED, BARRIER_MAX_FRAME_SPEED_TIMEOUT_MS);
  joint_state_pub_.stopPublishing();
}

TEST_F(JointStatesSpeedObserverTest, testMaxFrameSpeed)
{
  double max_frame_speed_target{ 0.3 };

  EXPECT_CALL(subscriber_mock_, cb_mock(MaxFrameSpeedLe(max_frame_speed_target))).Times(AtLeast(0));
  EXPECT_CALL(subscriber_mock_, cb_mock(MaxFrameSpeedNear(max_frame_speed_target)))
      .WillOnce(Return())
      .WillOnce(ACTION_OPEN_BARRIER_VOID(BARRIER_MAX_FRAME_SPEED))
      .WillRepeatedly(Return());

  double angular_velocity = max_frame_speed_target / RIGHT_ARM_RADIUS;
  joint_state_pub_.setJoint1Velocity(angular_velocity);

  BARRIER(BARRIER_MAX_FRAME_SPEED, BARRIER_MAX_FRAME_SPEED_TIMEOUT_MS);
}

}  // namespace joint_states_speed_observer_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unittest_joint_states_speed_observer");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 2 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
