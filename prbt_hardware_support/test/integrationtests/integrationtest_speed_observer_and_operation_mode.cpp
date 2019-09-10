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
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>

#include <pilz_testutils/async_test.h>
#include <prbt_hardware_support/ros_test_helper.h>
#include <prbt_hardware_support/speed_observer.h>
#include <prbt_hardware_support/FrameSpeeds.h>
#include <prbt_hardware_support/GetOperationMode.h>
#include <prbt_hardware_support/OperationModes.h>
#include <prbt_hardware_support/wait_for_service.h>

namespace speed_observer_test
{
using ::testing::_;
using ::testing::AtLeast;
using ::testing::PrintToString;
using ::testing::Return;
using ::testing::SetArgReferee;
using ::testing::Sequence;
using namespace prbt_hardware_support;

static const std::string FRAME_SPEEDS_TOPIC_NAME{ "/frame_speeds" };
static const std::string FAKE_CONTROLLER_JOINT_STATES_TOPIC_NAME{ "/fake_controller_joint_states" };
static const std::string OPERATION_MODE_TOPIC{"operation_mode"};
static const std::string STO_SERVICE{ "safe_torque_off" };
static const std::string OPERATION_MODE_SERVICE{ "/prbt/get_operation_mode" };
static const std::string ADDITIONAL_FRAMES_PARAM_NAME{ "additional_frames" };

static const std::string BARRIER_OPERATION_MODE_SET{ "BARRIER_OPERATION_MODE_SET" };
static const std::string BARRIER_STOP_HAPPENED{ "BARRIER_STOP_HAPPENED" };
static const std::string BARRIER_NO_STOP_HAPPENED{ "BARRIER_NO_STOP_HAPPENED" };
static const std::string BARRIER_NO_SVC_SUCESS{ "BARRIER_NO_SVC_SUCESS" };

static const double TEST_FREQUENCY{ 10 };

MATCHER_P(StoState, x, "Sto state " + std::string(negation ? "is not" : "is") + ": " + PrintToString(x) + ".")
{
  return arg.data == x;
}

MATCHER_P(ContainsAllNames, names,
          "Names" + PrintToString(names) + std::string(negation ? "are not" : "are") + " in message.")
{
  for (auto& name : names)
  {
    bool found{ false };
    for (auto& argn : arg->name)
    {
      if (argn.compare(name) == 0)
      {
        found = true;
      }
    }
    if (!found)
    {
      return false;
    }
  }

  return true;
}

class SpeedObserverIntegrationTest : public testing::Test, public testing::AsyncTest
{
public:
  void SetUp() override;
  void TearDown() override;

  MOCK_METHOD2(operation_mode_cb, bool(GetOperationMode::Request& req, GetOperationMode::Response& res));
  MOCK_METHOD2(sto_cb, bool(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res));
  MOCK_METHOD1(frame_speeds_cb, void(const FrameSpeeds::ConstPtr& msg));

  void publishJointStatesAtSpeed(double v);
  void stopJointStatePublisher();
  void publishOperationMode(OperationModes::_value_type omv);

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_{ "~" };
  ros::Subscriber subscriber_;
  ros::ServiceServer operation_mode_srv_;
  ros::ServiceServer sto_srv_;
  ros::Publisher fake_controller_joint_states_pub_;
  ros::Publisher operation_mode_pub_;
  std::vector<std::string> additional_frames_;
  bool joint_publisher_running_{ false };
  std_srvs::SetBool::Response sto_res;
};

void SpeedObserverIntegrationTest::SetUp()
{
  subscriber_ =
      nh_.subscribe<FrameSpeeds>(FRAME_SPEEDS_TOPIC_NAME, 1, &SpeedObserverIntegrationTest::frame_speeds_cb, this);
  fake_controller_joint_states_pub_ =
      nh_.advertise<sensor_msgs::JointState>(FAKE_CONTROLLER_JOINT_STATES_TOPIC_NAME, 1);
  operation_mode_pub_ =
      nh_.advertise<OperationModes>(OPERATION_MODE_TOPIC, 10);

  pnh_.getParam(ADDITIONAL_FRAMES_PARAM_NAME, additional_frames_);
  ROS_DEBUG_STREAM("SetUp pnh:" << pnh_.getNamespace() << " nh:" << nh_.getNamespace());
  for (auto& frame : additional_frames_)
  {
    ROS_DEBUG_STREAM("- " << frame);
  }

  operation_mode_srv_ =
      nh_.advertiseService(OPERATION_MODE_SERVICE, &SpeedObserverIntegrationTest::operation_mode_cb, this);
  sto_srv_ = nh_.advertiseService(STO_SERVICE, &SpeedObserverIntegrationTest::sto_cb, this);

  sto_res.success = true;
  sto_res.message = "testing ...";
}

void SpeedObserverIntegrationTest::TearDown()
{
  joint_publisher_running_ = false;
  nh_.shutdown();
}

void SpeedObserverIntegrationTest::publishOperationMode(OperationModes::_value_type omv){
  OperationModes om;
  om.value = omv;
  om.time_stamp = ros::Time::now();
  operation_mode_pub_.publish(om);
}

void SpeedObserverIntegrationTest::publishJointStatesAtSpeed(double speed)
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
    js.position = { t * speed };

    fake_controller_joint_states_pub_.publish(js);
    if (joint_publisher_running_ & nh_.ok())  // ending fasteroperation_mode_cb
      r.sleep();
  }
}

void SpeedObserverIntegrationTest::stopJointStatePublisher()
{
  joint_publisher_running_ = false;
}

/**
 * @tests{Stop1_on_violation_of_speed_limit}
 *
 * Test Sequence:
 *    0. Set Operation Mode to T1 by answering the service call
 *    1. Joint motion with a speed higher than the T1 limit is faked.
 *    2. Set Operation Mode to Auto by publihsing on the topic
 *       Wait for stop calls that may occurr from previous run
 *    3. Joint motion with a speed lower than the Auto limit is faked.
 *    4. Switching back to T1
 *    5. Set up the STO service to return success = false
 *
 * Expected Results:
 *    0. -
 *    1. Operation mode not to be called any more
 *       Sto is triggered
 *    2. -
 *    3. Operation mode not to be called any more
 *       Sto is *not* triggered
 *    4. Sto is triggered
 *    5. Sto is triggered and error message is produced
 */
TEST_F(SpeedObserverIntegrationTest, testT1)
{
  Sequence s_speeds, s_stop;
  /**********
   * Step 0 *
   **********/
  ROS_DEBUG("Step 0");

  // Set OM to T1
  GetOperationMode::Response om_res;
  om_res.mode.value = OperationModes::T1;
  om_res.mode.time_stamp = ros::Time::now();
  EXPECT_CALL(*this, operation_mode_cb(_, _)).InSequence(s_stop)
      .WillOnce(DoAll(SetArgReferee<1>(om_res), ACTION_OPEN_BARRIER(BARRIER_OPERATION_MODE_SET)));
  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_))).Times(AtLeast(1)).InSequence(s_speeds);
  EXPECT_CALL(*this, sto_cb(_, _)).Times(0);

  BARRIER({ BARRIER_OPERATION_MODE_SET });

  /**********
   * Step 1 *
   **********/
  ROS_DEBUG("Step 1");

  // Now the service should not be called any more
  EXPECT_CALL(*this, operation_mode_cb(_, _)).Times(0).InSequence(s_stop);
  EXPECT_CALL(*this, sto_cb(StoState(true), _))
      .WillRepeatedly(DoAll(SetArgReferee<1>(sto_res), ACTION_OPEN_BARRIER(BARRIER_STOP_HAPPENED)));

  std::thread pubisher_thread = std::thread(&SpeedObserverIntegrationTest::publishJointStatesAtSpeed, this, 1.0);
  BARRIER({ BARRIER_STOP_HAPPENED });
  stopJointStatePublisher();
  pubisher_thread.join();

  /**********
   * Step 2 *
   **********/
  ROS_DEBUG("Step 2");

  EXPECT_CALL(*this, sto_cb(StoState(true), _)).InSequence(s_stop)
      .WillRepeatedly(SetArgReferee<1>(sto_res));
  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_))).Times(AtLeast(2)).InSequence(s_speeds);
  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_))).InSequence(s_speeds).WillOnce(ACTION_OPEN_BARRIER_VOID(BARRIER_OPERATION_MODE_SET));

  // Set OM to Auto
  publishOperationMode(OperationModes::AUTO);
  BARRIER({ BARRIER_OPERATION_MODE_SET });

  /**********
   * Step 3 *
   **********/
  ROS_DEBUG("Step 3");

  EXPECT_CALL(*this, sto_cb(StoState(true), _)).InSequence(s_stop)
      .WillOnce(SetArgReferee<1>(sto_res));  // may be called once at start of publisher
  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_))).Times(AtLeast(2)).InSequence(s_speeds);
  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_))).InSequence(s_speeds).WillOnce(ACTION_OPEN_BARRIER_VOID(BARRIER_NO_STOP_HAPPENED));

  pubisher_thread = std::thread(&SpeedObserverIntegrationTest::publishJointStatesAtSpeed, this, 1.0);
  BARRIER({ BARRIER_NO_STOP_HAPPENED });

  /**********
   * Step 4 *
   **********/
  ROS_DEBUG("Step 4");

  EXPECT_CALL(*this, sto_cb(StoState(true), _)).InSequence(s_stop)
      .WillRepeatedly(DoAll(SetArgReferee<1>(sto_res), ACTION_OPEN_BARRIER(BARRIER_STOP_HAPPENED)));
  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_))).Times(AtLeast(1)).InSequence(s_speeds);

  publishOperationMode(OperationModes::T1);
  BARRIER({ BARRIER_STOP_HAPPENED });

  /**********
   * Step 5 *
   **********/
  ROS_DEBUG("Step 5");

  sto_res.success = false;
  EXPECT_CALL(*this, sto_cb(StoState(true), _)).InSequence(s_stop)
      .WillRepeatedly(DoAll(SetArgReferee<1>(sto_res), ACTION_OPEN_BARRIER(BARRIER_NO_SVC_SUCESS)));

  BARRIER({BARRIER_NO_SVC_SUCESS});
  stopJointStatePublisher();
  pubisher_thread.join();
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
