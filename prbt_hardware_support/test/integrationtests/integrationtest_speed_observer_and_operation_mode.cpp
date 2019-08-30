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

#include <pilz_testutils/async_test.h>
#include <prbt_hardware_support/ros_test_helper.h>
#include <prbt_hardware_support/speed_observer.h>
#include <prbt_hardware_support/FrameSpeeds.h>
#include <prbt_hardware_support/GetOperationMode.h>

namespace speed_observer_test
{
using ::testing::_;
using namespace prbt_hardware_support;

static const std::string FRAME_SPEEDS_TOPIC_NAME{ "/frame_speeds" };
static const std::string STOP_TOPIC_NAME{ "/stop" };
static const std::string ADDITIONAL_FRAMES_PARAM_NAME{ "additional_frames" };
static const std::string OPERATION_MODE_SERVICE{ "/prbt/get_operation_mode" };

class SpeedObserverIntegarionTest : public testing::Test,
                                    public testing::AsyncTest
{
public:
  void SetUp() override;
  //  virtual void TearDown();

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_{ "~" };
  ros::Subscriber subscriber_;
  ros::ServiceServer operation_mode_srv_;
  FrameSpeeds last_frame_speeds_;
  std::vector<std::string> additional_frames_;

  MOCK_METHOD2(operation_mode_cb, bool(GetOperationMode::Request& req,
                                       GetOperationMode::Response& res));
  MOCK_METHOD1(frame_speeds_cb, void(FrameSpeeds msg));
};

void SpeedObserverIntegarionTest::SetUp()
{
  last_frame_speeds_ = FrameSpeeds();
  subscriber_ = nh_.subscribe<FrameSpeeds>(
      FRAME_SPEEDS_TOPIC_NAME, 1, &SpeedObserverIntegarionTest::frame_speeds_cb,
      this);
  operation_mode_srv_ = nh_.advertiseService(
      OPERATION_MODE_SERVICE, &SpeedObserverIntegarionTest::operation_mode_cb,
      this);

  pnh_.getParam(ADDITIONAL_FRAMES_PARAM_NAME, additional_frames_);
  ROS_DEBUG_STREAM("SetUp pnh:" << pnh_.getNamespace()
                                << " nh:" << nh_.getNamespace());
  for (auto& f : additional_frames_)
  {
    ROS_DEBUG_STREAM("- " << f);
  }
}

void SpeedObserverIntegarionTest::frameSpeedsCb(
    const FrameSpeeds::ConstPtr& msg)
{
  last_frame_speeds_ = FrameSpeeds(*msg);
}

// void SpeedObserverIntegarionTest::TearDown(){}

/**
 * @tests{The correct operation of the node and interpretion of params}
 *
 * Test Sequence:
 *    0.
 *
 * Expected Results:
 *    0. -
 */
TEST_F(SpeedObserverIntegarionTest, testStartupAndTopic)
{
  /**********
   * Setup *
   **********/
  waitForNode("/robot_state_publisher");
  waitForNode("/speed_observer_node");
  while (last_frame_speeds_.name.empty())
  {
    ros::Duration(.1).sleep();
    ros::spinOnce();
  }
  for (auto& n : last_frame_speeds_.name)
  {
    ROS_DEBUG_STREAM("> " << n);
  }
  //  EXPECT_THAT(last_frame_speeds_.name, ::testing::Contains("world"));
  BARRIER({ "you shall not pass" });
}

}  // namespace speed_observer_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unittest_speed_observer");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
