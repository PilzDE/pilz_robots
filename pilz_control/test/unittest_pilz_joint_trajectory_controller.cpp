/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <ros/package.h>

#include <controller_interface/controller_base.h>
#include <trajectory_interface/quintic_spline_segment.h>

#include <pilz_control/pilz_joint_trajectory_controller.h>

typedef hardware_interface::JointCommandInterface HWInterface;
typedef trajectory_interface::QuinticSplineSegment<double> Segment;
typedef pilz_joint_trajectory_controller::PilzJointTrajectoryController<Segment, HWInterface> Controller;

namespace pilz_joint_trajectory_controller
{

/**
 * @brief Test fixture class for the unit-test of the PilzJointTrajectoryController.
 *
 * A minimal test-robot is used to initialize the controller.
 */
class PilzJointTrajectoryControllerTest : public testing::Test
{
protected:
  void SetUp();

protected:
  std::shared_ptr<Controller> controller_;
  HWInterface* hardware_ {new HWInterface()};
  ros::NodeHandle nh_ {"~"};
  ros::NodeHandle controller_nh_ {"controller_ns"};
  ros::AsyncSpinner spinner_ {2};
};

void PilzJointTrajectoryControllerTest::SetUp()
{
  spinner_.start();

  // register joint handle
  double* pos = new double();
  double* vel = new double();
  double* eff = new double();
  hardware_interface::JointStateHandle jsh {"joint1", pos, vel, eff};
  double* cmd = new double();
  hardware_interface::JointHandle jh {jsh, cmd};
  hardware_->registerHandle(jh);

  // set joints parameter
  controller_nh_.setParam("joints", std::vector<std::string>( {"joint1"} ));

  // Setup controller
  controller_ = std::make_shared<Controller>();
}

/**
 * @brief Test the initialization of the PilzJointTrajectoryController.
 *
 * Test Sequence:
 *    1. Construct the controller and call init().
 *
 * Expected Results:
 *    1. The force hold service exists and the controller is in mode HOLDING.
 */
TEST_F(PilzJointTrajectoryControllerTest, testInitializiation)
{
  controller_ = std::make_shared<Controller>();

  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";

  ASSERT_TRUE(ros::service::exists(controller_nh_.getNamespace() + "/hold", true));
  ASSERT_TRUE(ros::service::exists(controller_nh_.getNamespace() + "/unhold", true));
}

/**
 * @brief Test the force_hold service callback of the PilzJointTrajectoryController.
 *
 * Test Sequence:
 *    1. Call the callback with a request to leave HOLDING.
 *    2. Call the callback with a request to enter HOLDING.
 *
 * Expected Results:
 *    1. Controller is in mode DEFAULT.
 *    2. Controller is in mode HOLDING.
 */
TEST_F(PilzJointTrajectoryControllerTest, testForceHoldServiceCallback)
{
  controller_ = std::make_shared<Controller>();

  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  ASSERT_TRUE(controller_->handleUnHoldRequest(req, resp));

  ASSERT_TRUE(controller_->handleHoldRequest(req, resp));
}


/**
 * @brief Test for correct return values in case of subsequent calls to hold/unhold function.
 *
 * Test Sequence:
 *    1. Request HOLDING twice.
 *    2. Request DEFAULT twice.
 *
 * Expected Results:
 *    1. Response contains success==True.
 *    2. Response contains success==True.
 */
TEST_F(PilzJointTrajectoryControllerTest, testDoubleRequest)
{
  controller_ = std::make_shared<Controller>();

  ASSERT_TRUE(controller_->init(hardware_, nh_, controller_nh_)) << "Failed to initialize the controller.";

  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  ASSERT_TRUE(controller_->handleUnHoldRequest(req, resp));
  EXPECT_TRUE(resp.success);
  ASSERT_TRUE(controller_->handleUnHoldRequest(req, resp));
  EXPECT_TRUE(resp.success);

  ASSERT_TRUE(controller_->handleHoldRequest(req, resp));
  EXPECT_TRUE(resp.success);
  ASSERT_TRUE(controller_->handleHoldRequest(req, resp));
  EXPECT_TRUE(resp.success);
}

}  // namespace pilz_joint_trajectory_controller


int main(int argc, char **argv)
{
  ros::init(argc, argv, "unittest_pilz_joint_trajectory_controller");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
