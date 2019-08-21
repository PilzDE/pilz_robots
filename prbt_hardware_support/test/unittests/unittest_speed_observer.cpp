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
#include <tf2_ros/transform_broadcaster.h>

#include <prbt_hardware_support/ros_test_helper.h>
#include <prbt_hardware_support/speed_observer.h>
#include <prbt_hardware_support/FrameSpeeds.h>

namespace speed_observer_test
{
using ::testing::_;
using namespace prbt_hardware_support;

static const std::string FRAME_SPEEDS_TOPIC_NAME{"/frame_speeds"};
static const std::string STOP_TOPIC_NAME{"/stop"};
static const double TEST_FREQUENCY{10};
static const std::string TEST_BASE_FRAME{"test_base"};
static const std::string TEST_FRAME_A{"a"};
static const std::string TEST_FRAME_B{"b"};
static const double SQRT_2{1 / sqrt(2)};
static const double PI_2{2 * M_PI};

class SpeedObserverIntegarionTest : public testing::Test
{

public:
  void SetUp() override;
  void TearDown() override;

  MOCK_METHOD1(frame_speeds_cb_mock, void(FrameSpeeds msg));
  void frameSpeedsCb(const FrameSpeeds::ConstPtr& msg);
  void publishTfWithSpeed(double v, double t);

protected:
  ros::Subscriber subscriber_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_{"~"};
  FrameSpeeds last_frame_speeds_;
  std::vector<std::string> additional_frames_;
};

void SpeedObserverIntegarionTest::SetUp(){
  last_frame_speeds_ = FrameSpeeds();
  subscriber_ = nh_.subscribe<FrameSpeeds>(
        FRAME_SPEEDS_TOPIC_NAME,
        1,
        &SpeedObserverIntegarionTest::frameSpeedsCb,
        this);
}

void SpeedObserverIntegarionTest::TearDown(){}

void SpeedObserverIntegarionTest::frameSpeedsCb(const FrameSpeeds::ConstPtr& msg){
  last_frame_speeds_ = FrameSpeeds(*msg);
}

void SpeedObserverIntegarionTest::publishTfWithSpeed(double v, double duration_s){
  static tf2_ros::TransformBroadcaster br;
  ros::Rate r = ros::Rate(TEST_FREQUENCY * 3);
  ros::Time start = ros::Time::now();
  double t = 0;
  while(t < duration_s){
    ros::Time current = ros::Time::now();
    t = (current-start).toSec();
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
    // rotation in a tilted circle
    tfsb.transform.translation.x = v / PI_2 * cos(t);
    tfsb.transform.translation.y = v / PI_2 * SQRT_2 * -sin(t);
    tfsb.transform.translation.z = v / PI_2 * SQRT_2 * sin(t);
    tfsb.transform.rotation.w = 1;
    br.sendTransform(tfsa);
    br.sendTransform(tfsb);
    r.sleep();
  }
}

/**
 * @tests{The correct observation of frames}
 *
 * Test Sequence:
 *    0. Starting up
 *
 * Expected Results:
 *    0. -
 */
TEST_F(SpeedObserverIntegarionTest, testStartupAndTopic)
{
  ROS_DEBUG_STREAM("test started");
  /**********
   * Setup *
   **********/
  std::string reference_frame{TEST_BASE_FRAME};
  std::vector<std::string> frames_to_observe{TEST_FRAME_A, TEST_FRAME_B};
  prbt_hardware_support::SpeedObserver observer(
    nh_,
    reference_frame,
    frames_to_observe
  );
  //  observer.startObserving(TEST_FREQUENCY);
  std::thread observer_thread = std::thread(&SpeedObserver::startObserving, &observer, TEST_FREQUENCY);
  ROS_DEBUG_STREAM("thread started");

  /**********
   * Step 0 *
   **********/
  // rotating a multiple of 2pi to end up at the start
  std::thread pubisher_thread_ = std::thread(&SpeedObserverIntegarionTest::publishTfWithSpeed, this, 0, PI_2);
  while(last_frame_speeds_.name.empty()){
    ros::Duration(.1).sleep();
    ros::spinOnce();
  }
  for(auto & n : last_frame_speeds_.name){
    ROS_DEBUG_STREAM("> " << n);
  }
  EXPECT_THAT(last_frame_speeds_.name, ::testing::Contains(TEST_FRAME_A));
  EXPECT_THAT(last_frame_speeds_.name, ::testing::Contains(TEST_FRAME_B));
  pubisher_thread_.join();

  /*************
   * Tear Down *
   *************/
  observer.terminateNow();
  observer_thread.join();
}

} // namespace speed_observer_test

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_speed_observer");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

