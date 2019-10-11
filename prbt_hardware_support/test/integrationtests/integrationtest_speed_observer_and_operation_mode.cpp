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
#include <tf2_ros/transform_broadcaster.h>

#include <pilz_testutils/async_test.h>
#include <prbt_hardware_support/ros_test_helper.h>
#include <prbt_hardware_support/speed_observer.h>
#include <prbt_hardware_support/FrameSpeeds.h>
#include <prbt_hardware_support/OperationModes.h>

namespace speed_observer_test
{
using ::testing::_;
using ::testing::AtLeast;
using ::testing::InSequence;
using ::testing::PrintToString;
using ::testing::Return;
using ::testing::SetArgReferee;
using namespace prbt_hardware_support;

static const std::string FRAME_SPEEDS_TOPIC_NAME{ "/frame_speeds" };
static const std::string FAKE_CONTROLLER_JOINT_STATES_TOPIC_NAME{ "/fake_controller_joint_states" };
static const std::string OPERATION_MODE_TOPIC{ "operation_mode" };
static const std::string STO_SERVICE{ "safe_torque_off" };
static const std::string ADDITIONAL_FRAMES_PARAM_NAME{ "additional_frames" };
static const std::string SPEED_LIMIT_T1_PARAM_NAME{ "speed_limit_t1" };
static const std::string SPEED_LIMIT_AUTOMATIC_PARAM_NAME{ "speed_limit_automatic" };

static const std::string BARRIER_STOP_HAPPENED{ "BARRIER_STOP_HAPPENED" };
static const std::string BARRIER_NO_STOP_HAPPENED{ "BARRIER_NO_STOP_HAPPENED" };
static const std::string BARRIER_NO_SVC_SUCESS{ "BARRIER_NO_SVC_SUCESS" };
static const std::string BARRIER_WAIT_OUT_STOP{ "BARRIER_WAIT_OUT_STOP" };

static const double SQRT_2_HALF{ 1 / sqrt(2) };
static const double TEST_FREQUENCY{ 10 };
static const std::string TEST_WORLD_FRAME{ "world" };

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

  MOCK_METHOD2(sto_cb, bool(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res));
  MOCK_METHOD1(frame_speeds_cb, void(const FrameSpeeds::ConstPtr& msg));

  void publishTfAtSpeed(double speed, const std::string& frame);
  void publishJointStatesAtSpeed(double v);
  void stopJointStatePublisher();
  void stopTfPublisher();
  void publishOperationMode(OperationModes::_value_type omv);

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_{ "~" };
  ros::AsyncSpinner spinner_{2};
  ros::Subscriber subscriber_;
  ros::ServiceServer sto_srv_;
  ros::Publisher fake_controller_joint_states_pub_;
  ros::Publisher operation_mode_pub_;
  std::vector<std::string> additional_frames_;
  bool joint_publisher_running_{ false };
  bool tf_publisher_running_{ false };
  std_srvs::SetBool::Response sto_res;
  double speed_limit_t1_;
  double speed_limit_automatic_;
};

void SpeedObserverIntegrationTest::SetUp()
{
  spinner_.start();

  subscriber_ =
      nh_.subscribe<FrameSpeeds>(FRAME_SPEEDS_TOPIC_NAME, 1, &SpeedObserverIntegrationTest::frame_speeds_cb, this);
  fake_controller_joint_states_pub_ =
      nh_.advertise<sensor_msgs::JointState>(FAKE_CONTROLLER_JOINT_STATES_TOPIC_NAME, 1);
  operation_mode_pub_ = nh_.advertise<OperationModes>(OPERATION_MODE_TOPIC, 10, true);

  ASSERT_TRUE(pnh_.getParam(SPEED_LIMIT_T1_PARAM_NAME, speed_limit_t1_));
  ASSERT_TRUE(pnh_.getParam(SPEED_LIMIT_AUTOMATIC_PARAM_NAME, speed_limit_automatic_));
  ASSERT_TRUE(pnh_.getParam(ADDITIONAL_FRAMES_PARAM_NAME, additional_frames_));
  ROS_DEBUG_STREAM("SetUp pnh:" << pnh_.getNamespace() << " nh:" << nh_.getNamespace());
  for (auto& frame : additional_frames_)
  {
    ROS_DEBUG_STREAM("- " << frame);
  }

  sto_srv_ = nh_.advertiseService(STO_SERVICE, &SpeedObserverIntegrationTest::sto_cb, this);

  sto_res.success = true;
  sto_res.message = "testing ...";

  waitForNode("/speed_observer_node");
  waitForNode("/operation_mode_setup_executor_node");
}

void SpeedObserverIntegrationTest::TearDown()
{
  joint_publisher_running_ = false;
  spinner_.stop();
  nh_.shutdown();
}

void SpeedObserverIntegrationTest::publishTfAtSpeed(double speed, const std::string& frame)
{
  static tf2_ros::TransformBroadcaster br;
  ros::Rate r = ros::Rate(TEST_FREQUENCY * 3); // publishing definitely faster then observing
  ros::Time start = ros::Time::now();
  double t = 0;
  tf_publisher_running_ = true;
  while (tf_publisher_running_)
  {
    ros::Time current = ros::Time::now();
    t = (current - start).toSec();
    geometry_msgs::TransformStamped tranform_stamped_b;
    tranform_stamped_b.header.stamp = current + ros::Duration(0.1); // avoid override by static_transform_publisher
    tranform_stamped_b.header.frame_id = TEST_WORLD_FRAME;
    tranform_stamped_b.child_frame_id = frame;
    // rotation in a tilted circle to cover all axes
    tranform_stamped_b.transform.translation.x = speed * cos(t);
    tranform_stamped_b.transform.translation.y = speed * SQRT_2_HALF * -sin(t);
    tranform_stamped_b.transform.translation.z = speed * SQRT_2_HALF * sin(t);
    tranform_stamped_b.transform.rotation.w = 1;
    br.sendTransform(tranform_stamped_b);

    if (tf_publisher_running_)  // ending faster
      r.sleep();
  }
}

void SpeedObserverIntegrationTest::publishOperationMode(OperationModes::_value_type omv)
{
  OperationModes om;
  om.value = omv;
  om.time_stamp = ros::Time::now();
  operation_mode_pub_.publish(om);
}

void SpeedObserverIntegrationTest::publishJointStatesAtSpeed(double speed)
{
  sensor_msgs::JointState js;
  js.name = { "testing_world-a" };
  ros::Rate r = ros::Rate(TEST_FREQUENCY * 3); // publishing definitely faster
                                               // then observing
  ros::Time start = ros::Time::now();
  double t = 0;
  double t_zero = 0;
  joint_publisher_running_ = true;
  bool first_round{true};
  while (joint_publisher_running_ & nh_.ok())
  {
    ros::Time current = ros::Time::now();
    t = (current - start).toSec();
    if (first_round)
    {
      t_zero = t;
      first_round = false;
    }
    // Starting with max speed abruptly is not a relevant scenario and makes the test instable
    // The following realizes half the speed in the first round and later constant full speed
    js.position = { (t - 0.5*t_zero) * speed };

    fake_controller_joint_states_pub_.publish(js);
    if (joint_publisher_running_ & nh_.ok())  // ending faster
      r.sleep();
  }
}

void SpeedObserverIntegrationTest::stopJointStatePublisher()
{
  joint_publisher_running_ = false;
}

void SpeedObserverIntegrationTest::stopTfPublisher()
{
  tf_publisher_running_ = false;
}

/**
 * @brief Tests speed observer with operation mode T1.
 *
 * @tests{Monitor_Speed_of_all_tf_frames_until_TCP,
 * Tests that robot model is read correctly by providing a custom xacro.
 * }
 * @tests{Monitor_Speed_of_user_defined_tf_frames,
 * Tests that user defined frames are monitored correctly.
 * }
 * @tests{Stop1_on_violation_of_speed_limit,
 * Tests that Stop 1 is triggered if speed limit is violated.
 * }
 * @tests{Speed_limits_per_operation_mode,
 * Tests the existence of the right limits by setting the mode and
 * moving in speeds belowe and above the expected limit.
 * }
 *
 * Test Sequence:
 *    1. Publish Operation Mode T1.
 *    2. Joint motion with a speed higher than the T1 limit is faked.
 *
 * Expected Results:
 *    1. Sto is *not* triggered
 *    2. Sto is triggered
 */
TEST_F(SpeedObserverIntegrationTest, testOperationModeT1)
{
  /**********
   * Step 1 *
   **********/
  ROS_DEBUG("Step 1");

  EXPECT_CALL(*this, sto_cb(_, _)).Times(0);

  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_)))
      .WillOnce(Return())
      .WillOnce(Return())
      .WillOnce(ACTION_OPEN_BARRIER_VOID(BARRIER_NO_STOP_HAPPENED))
      .WillRepeatedly(Return());

  // Set OM to T1
  publishOperationMode(OperationModes::T1);

  BARRIER({ BARRIER_NO_STOP_HAPPENED });

  /**********
   * Step 2 *
   **********/
  ROS_DEBUG("Step 2");

  EXPECT_CALL(*this, sto_cb(StoState(false), _))
      .WillOnce(DoAll(SetArgReferee<1>(sto_res), ACTION_OPEN_BARRIER(BARRIER_STOP_HAPPENED)))
      .WillRepeatedly(DoAll(SetArgReferee<1>(sto_res), Return(true)));

  std::thread pubisher_thread = std::thread(&SpeedObserverIntegrationTest::publishJointStatesAtSpeed, this,
                                            speed_limit_t1_ + 0.01);
  BARRIER({ BARRIER_STOP_HAPPENED });
  stopJointStatePublisher();
  pubisher_thread.join();

  // Wait for more iterations of the speed observer, enabling it to process the present change of tfs
  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_)))
      .WillOnce(Return())
      .WillOnce(ACTION_OPEN_BARRIER_VOID(BARRIER_WAIT_OUT_STOP))
      .WillRepeatedly(Return());

  BARRIER({ BARRIER_WAIT_OUT_STOP });
}

/**
 * @brief Tests speed observer with operation mode AUTO.
 *
 * @tests{Monitor_Speed_of_all_tf_frames_until_TCP,
 * Tests that robot model is read correctly by providing a custom xacro.
 * }
 * @tests{Monitor_Speed_of_user_defined_tf_frames,
 * Tests that user defined frames are monitored correctly.
 * }
 * @tests{Stop1_on_violation_of_speed_limit,
 * Tests that Stop 1 is triggered if speed limit is violated.
 * }
 * @tests{Speed_limits_per_operation_mode,
 * Tests the existence of the right limits by setting the mode and
 * moving in speeds belowe and above the expected limit.
 * }
 *
 * Test Sequence:
 *    1. Publish Operation Mode AUTO.
 *    2. Joint motion with a speed lower than the Auto limit is faked.
 *    3. Publish Operation Mode T1.
 *
 * Expected Results:
 *    1. Sto is *not* triggered
 *    2. Sto is *not* triggered
 *    3. Sto is triggered
 */
TEST_F(SpeedObserverIntegrationTest, testOperationModeAuto)
{
  /**********
   * Step 1 *
   **********/
  ROS_DEBUG("Step 1");

  EXPECT_CALL(*this, sto_cb(_, _)).Times(0);

  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_)))
      .WillOnce(Return())
      .WillOnce(Return())
      .WillOnce(ACTION_OPEN_BARRIER_VOID(BARRIER_NO_STOP_HAPPENED))
      .WillRepeatedly(Return());

  // Set OM to Auto
  publishOperationMode(OperationModes::AUTO);
  BARRIER({ BARRIER_NO_STOP_HAPPENED });

  /**********
   * Step 2 *
   **********/
  ROS_DEBUG("Step 2");

  EXPECT_CALL(*this, sto_cb(_, _)).Times(0);

  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_)))
      .WillOnce(Return())
      .WillOnce(Return())
      .WillOnce(ACTION_OPEN_BARRIER_VOID(BARRIER_NO_STOP_HAPPENED))
      .WillRepeatedly(Return());

  std::thread pubisher_thread = std::thread(&SpeedObserverIntegrationTest::publishJointStatesAtSpeed, this,
                                            speed_limit_automatic_ - 0.01);
  BARRIER({ BARRIER_NO_STOP_HAPPENED });

  /**********
   * Step 3 *
   **********/
  ROS_DEBUG("Step 3");

  EXPECT_CALL(*this, sto_cb(StoState(false), _))
      .WillOnce(DoAll(SetArgReferee<1>(sto_res), ACTION_OPEN_BARRIER(BARRIER_STOP_HAPPENED)))
      .WillRepeatedly(DoAll(SetArgReferee<1>(sto_res), Return(true)));

  publishOperationMode(OperationModes::T1);
  BARRIER({ BARRIER_STOP_HAPPENED });
  stopJointStatePublisher();
  pubisher_thread.join();

  // Wait for more iterations of the speed observer, enabling it to process the present change of tfs
  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_)))
      .WillOnce(Return())
      .WillOnce(ACTION_OPEN_BARRIER_VOID(BARRIER_WAIT_OUT_STOP))
      .WillRepeatedly(Return());

  BARRIER({ BARRIER_WAIT_OUT_STOP });
}

/**
 * @brief Tests speed observer with additional tf tree.
 *
 * @tests{Monitor_Speed_of_all_tf_frames_until_TCP,
 * Tests that robot model is read correctly by providing a custom xacro.
 * }
 * @tests{Monitor_Speed_of_user_defined_tf_frames,
 * Tests that user defined frames are monitored correctly.
 * }
 * @tests{Stop1_on_violation_of_speed_limit,
 * Tests that Stop 1 is triggered if speed limit is violated.
 * }
 * @tests{Speed_limits_per_operation_mode,
 * Tests the existence of the right limits by setting the mode and
 * moving in speeds belowe and above the expected limit.
 * }
 *
 * Test Sequence:
 *    1. Publish Operation Mode T1.
 *       Publish the additionally defined TF tree at speed.
 *
 * Expected Results:
 *    1. Sto is triggered
 */
TEST_F(SpeedObserverIntegrationTest, testAdditionalTFTree)
{
  ROS_DEBUG("Step 1");

  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_)))
      .WillRepeatedly(Return());

  EXPECT_CALL(*this, sto_cb(StoState(false), _))
      .WillOnce(DoAll(SetArgReferee<1>(sto_res), ACTION_OPEN_BARRIER(BARRIER_STOP_HAPPENED)))
      .WillRepeatedly(DoAll(SetArgReferee<1>(sto_res), Return(true)));

  publishOperationMode(OperationModes::T1);

  std::thread pubisher_thread = std::thread(&SpeedObserverIntegrationTest::publishTfAtSpeed,
                                            this,
                                            speed_limit_t1_ + 0.01,
                                            additional_frames_[0]);
  BARRIER({ BARRIER_STOP_HAPPENED });
  stopTfPublisher();
  pubisher_thread.join();

  // Wait for more iterations of the speed observer, enabling it to process the present change of tfs
  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_)))
      .WillOnce(Return())
      .WillOnce(ACTION_OPEN_BARRIER_VOID(BARRIER_WAIT_OUT_STOP))
      .WillRepeatedly(Return());

  BARRIER({ BARRIER_WAIT_OUT_STOP });
}

/**
 * @brief Tests speed observer with STO service returning no success.
 *
 * @tests{Monitor_Speed_of_all_tf_frames_until_TCP,
 * Tests that robot model is read correctly by providing a custom xacro.
 * }
 * @tests{Monitor_Speed_of_user_defined_tf_frames,
 * Tests that user defined frames are monitored correctly.
 * }
 * @tests{Stop1_on_violation_of_speed_limit,
 * Tests that Stop 1 is triggered if speed limit is violated.
 * }
 * @tests{Speed_limits_per_operation_mode,
 * Tests the existence of the right limits by setting the mode and
 * moving in speeds belowe and above the expected limit.
 * }
 *
 * Test Sequence:
 *    1. Publish Operation Mode T1.
 *       Set up the STO service to return success = false.
 *
 * Expected Results:
 *    1. Sto is triggered and error message is produced
 */
TEST_F(SpeedObserverIntegrationTest, testStoServiceNoSuccess)
{
  /**********
   * Step 1 *
   **********/
  ROS_DEBUG("Step 1");

  sto_res.success = false;
  EXPECT_CALL(*this, sto_cb(StoState(false), _))
      .WillOnce(DoAll(SetArgReferee<1>(sto_res), ACTION_OPEN_BARRIER(BARRIER_NO_SVC_SUCESS)))
      .WillRepeatedly(DoAll(SetArgReferee<1>(sto_res), Return(true)));

  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_)))
      .WillOnce(Return())
      .WillOnce(Return())
      .WillOnce(ACTION_OPEN_BARRIER_VOID(BARRIER_NO_STOP_HAPPENED))
      .WillRepeatedly(Return());

  // Set OM to T1
  publishOperationMode(OperationModes::T1);

  std::thread pubisher_thread = std::thread(&SpeedObserverIntegrationTest::publishJointStatesAtSpeed, this,
                                            speed_limit_t1_ + 0.01);
  BARRIER({ BARRIER_NO_SVC_SUCESS });
  stopJointStatePublisher();
  pubisher_thread.join();

  // Wait for more iterations of the speed observer, enabling it to process the present change of tfs
  EXPECT_CALL(*this, frame_speeds_cb(ContainsAllNames(additional_frames_)))
      .WillOnce(Return())
      .WillOnce(ACTION_OPEN_BARRIER_VOID(BARRIER_WAIT_OUT_STOP))
      .WillRepeatedly(Return());

  BARRIER({ BARRIER_WAIT_OUT_STOP });
}

}  // namespace speed_observer_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unittest_speed_observer");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
