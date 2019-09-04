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

#include <gmock/gmock.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <pilz_testutils/async_test.h>
#include <prbt_hardware_support/ros_test_helper.h>
#include <prbt_hardware_support/speed_observer.h>
#include <prbt_hardware_support/FrameSpeeds.h>
#include <prbt_hardware_support/GetOperationMode.h>
#include <prbt_hardware_support/OperationModes.h>

namespace speed_observer_test
{
using ::testing::_;
using ::testing::Return;
using ::testing::SetArgReferee;
using namespace prbt_hardware_support;

static const std::string FRAME_SPEEDS_TOPIC_NAME{ "/frame_speeds" };
static const std::string FAKE_CONTROLLER_JOINT_STATES_TOPIC_NAME{ "/fake_"
                                                                  "controller_"
                                                                  "joint_"
                                                                  "states" };
static const std::string STOP_TOPIC_NAME{ "/stop" };

static const std::string OPERATION_MODE_SERVICE{ "/prbt/get_operation_mode" };

static const std::string ADDITIONAL_FRAMES_PARAM_NAME{ "additional_frames" };

static const std::string BARRIER_OPERATION_MODE_SET{ "BARRIER_OPERATION_MODE_"
                                                     "SET" };
static const std::string BARRIER_STOP_HAPPENED{ "BARRIER_STOP_HAPPENED" };

static const double TEST_FREQUENCY{ 10 };

class SpeedObserverIntegarionTest : public testing::Test,
                                    public testing::AsyncTest
{
public:
  void SetUp() override;
  void TearDown() override;

  MOCK_METHOD2(operation_mode_cb, bool(GetOperationMode::Request& req,
                                       GetOperationMode::Response& res));
  MOCK_METHOD1(frame_speeds_cb, void(FrameSpeeds msg));
  void publishJointStatesAtSpeed(double v);
  void stopJointStatePublisher();
  void advertiseOmService();

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_{ "~" };
  ros::Subscriber subscriber_;
  ros::ServiceServer operation_mode_srv_;
  ros::Publisher fake_controller_joint_states_pub_;
  std::vector<std::string> additional_frames_;
  bool joint_publisher_running_{ false };
};

void SpeedObserverIntegarionTest::SetUp()
{
  subscriber_ = nh_.subscribe<FrameSpeeds>(
      FRAME_SPEEDS_TOPIC_NAME, 1, &SpeedObserverIntegarionTest::frame_speeds_cb,
      this);
  fake_controller_joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>(
      FAKE_CONTROLLER_JOINT_STATES_TOPIC_NAME, 1);

  pnh_.getParam(ADDITIONAL_FRAMES_PARAM_NAME, additional_frames_);
  ROS_DEBUG_STREAM("SetUp pnh:" << pnh_.getNamespace()
                                << " nh:" << nh_.getNamespace());
  for (auto& f : additional_frames_)
  {
    ROS_DEBUG_STREAM("- " << f);
  }
}

void SpeedObserverIntegarionTest::TearDown()
{
  joint_publisher_running_ = false;
  nh_.shutdown();
}

void SpeedObserverIntegarionTest::advertiseOmService()
{
  operation_mode_srv_ = nh_.advertiseService(
      OPERATION_MODE_SERVICE, &SpeedObserverIntegarionTest::operation_mode_cb,
      this);
}

void SpeedObserverIntegarionTest::publishJointStatesAtSpeed(double v)
{
  sensor_msgs::JointState js;
  js.name = { "testing_world-a" };
  ros::Rate r = ros::Rate(TEST_FREQUENCY * 3);  // publishing definitely faster
                                                // then observing
  ros::Time start = ros::Time::now();
  double t = 0;
  joint_publisher_running_ = true;
  while (joint_publisher_running_ & nh_.ok())
  {
    ros::Time current = ros::Time::now();
    t = (current - start).toSec();
    js.position = { t * v };

    fake_controller_joint_states_pub_.publish(js);
    if (joint_publisher_running_ & nh_.ok())  // ending fasteroperation_mode_cb
      r.sleep();
  }
}

void SpeedObserverIntegarionTest::stopJointStatePublisher()
{
  joint_publisher_running_ = false;
}

/**
 * @tests{System behaviour in T1}
 *
 * Test Sequence:
 *    0. Set Operation Mode to T1 by answering the service call
 *
 * Expected Results:
 *    0. -
 *    1. Operation mode not to be called any more
 */
TEST_F(SpeedObserverIntegarionTest, testT1)
{
  /**********
   * Step 0 *
   **********/
  ROS_DEBUG("Step 0");

  // Set OM to T1
  GetOperationMode::Response omr;
  omr.mode.value = OperationModes::T1;
  EXPECT_CALL(*this, operation_mode_cb(_, _))
      .WillOnce(DoAll(SetArgReferee<1>(omr),
                      ACTION_OPEN_BARRIER(BARRIER_OPERATION_MODE_SET)));
  advertiseOmService();

  BARRIER({ BARRIER_OPERATION_MODE_SET });

  /**********
   * Step 1 *
   **********/
  ROS_DEBUG("Step 1");

  // Now the service should not be called any more
  EXPECT_CALL(*this, operation_mode_cb(_, _)).Times(0);

  publishJointStatesAtSpeed(1);
  BARRIER({ BARRIER_STOP_HAPPENED });
  stopJointStatePublisher();
}

}  // namespace speed_observer_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unittest_speed_observer");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
