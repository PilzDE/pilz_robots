/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
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

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <trajectory_interface/quintic_spline_segment.h>

#include <pilz_control/pilz_joint_trajectory_controller.h>
#include <pilz_control/pilz_joint_trajectory_controller_impl.h>

#include "pjtc_test_helper.h"

namespace pilz_joint_trajectory_controller_test
{
static const std::string CONTROLLER_NAMESPACE{ "/controller_ns" };

static const std::string JOINT_WITH_HAS_ACC_LIM_FALSE{ "joint_with_has_acc_lim_false" };
static const std::string JOINT_WITH_UNDEFINED_MAX_ACC{ "joint_with_undefined_max_acc" };
static const std::string JOINT_WITH_UNDEFINED_HAS_ACC_LIM{ "joint_with_undefined_has_acc_lim" };

using namespace pilz_joint_trajectory_controller;

using Segment = trajectory_interface::QuinticSplineSegment<double>;

/**
 * @brief Dummy needed for testGetJointAccelerationLimits.
 */
class DummySegmentImpl
{
public:
  struct State : public Segment::State
  {
  };
  typedef typename Segment::Scalar Scalar;
  typedef typename Segment::Time Time;
};

/**
 * @brief Dummy needed for testGetJointAccelerationLimits.
 */
class DummyHardwareInterface : public hardware_interface::RobotHW
{
public:
  struct MyResourceHandleType
  {
  };
  typedef typename DummyHardwareInterface::MyResourceHandleType ResourceHandleType;
  typedef typename hardware_interface::JointHandle JointHandle;
};

class GetJointAccelerationLimitsTest : public testing::Test
{
protected:
  void SetUp() override;
  void runGetJointAccelerationLimits(const ros::NodeHandle& nh, const std::vector<std::string>& joint_names);

protected:
  ros::NodeHandle controller_nh_{ CONTROLLER_NAMESPACE };
  std::vector<std::string> controller_joint_names_;
};

void GetJointAccelerationLimitsTest::SetUp()
{
  controller_joint_names_ = std::vector<std::string>(JOINT_NAMES.begin(), JOINT_NAMES.end());
  setControllerParameters(CONTROLLER_NAMESPACE);
}

void GetJointAccelerationLimitsTest::runGetJointAccelerationLimits(const ros::NodeHandle& nh,
                                                                   const std::vector<std::string>& joint_names)
{
  PilzJointTrajectoryController<DummySegmentImpl, DummyHardwareInterface>::getJointAccelerationLimits(nh, joint_names);
}

/**
 * @tests{Monitor_joint_accelerations,
 *   Test if parameters for acceleration limits are correctly read.
 * }
 */
TEST_F(GetJointAccelerationLimitsTest, testHasLimitsTrue)
{
  ros::NodeHandle limits_nh(controller_nh_, LIMITS_NAMESPACE);
  const std::vector<boost::optional<double>> acceleration_limits =
      PilzJointTrajectoryController<DummySegmentImpl, DummyHardwareInterface>::getJointAccelerationLimits(
          limits_nh, controller_joint_names_);
  EXPECT_EQ(controller_joint_names_.size(), acceleration_limits.size());
  EXPECT_TRUE(acceleration_limits.at(0));
  EXPECT_TRUE(acceleration_limits.at(1));
  EXPECT_DOUBLE_EQ(*acceleration_limits.at(0), MAX_JOINT_ACCELERATION);
  EXPECT_DOUBLE_EQ(*acceleration_limits.at(1), MAX_JOINT_ACCELERATION);
}

/**
 * @tests{Monitor_joint_accelerations,
 *   Test if parameters for acceleration limits are omitted for joints which have no limit.
 * }
 */
TEST_F(GetJointAccelerationLimitsTest, testHasLimitsFalse)
{
  ros::NodeHandle nh{ "~" };
  std::stringstream full_param_name;
  full_param_name << JOINT_WITH_HAS_ACC_LIM_FALSE << "/" << HAS_ACCELERATION_PARAMETER;
  nh.setParam(full_param_name.str(), false);

  const std::vector<boost::optional<double>> acceleration_limits_has_acc_lim_false =
      PilzJointTrajectoryController<DummySegmentImpl, DummyHardwareInterface>::getJointAccelerationLimits(
          nh, { JOINT_WITH_HAS_ACC_LIM_FALSE });
  EXPECT_EQ(1U, acceleration_limits_has_acc_lim_false.size());
  EXPECT_FALSE(acceleration_limits_has_acc_lim_false.at(0));
}

/**
 * @tests{Monitor_joint_accelerations,
 *   Test if correct errors are thrown if parameters can not be correctly read.
 * }
 */
TEST_F(GetJointAccelerationLimitsTest, testUndefinedLimit)
{
  ros::NodeHandle nh{ "~" };
  std::stringstream full_param_name;
  full_param_name << JOINT_WITH_UNDEFINED_MAX_ACC << "/" << HAS_ACCELERATION_PARAMETER;
  nh.setParam(full_param_name.str(), true);

  EXPECT_THROW(runGetJointAccelerationLimits(nh, { JOINT_WITH_UNDEFINED_MAX_ACC }), ros::InvalidParameterException);
}

/**
 * @tests{Monitor_joint_accelerations,
 *   Test if correct errors are thrown if parameters can not be correctly read.
 * }
 */
TEST_F(GetJointAccelerationLimitsTest, testUndefinedHasLimits)
{
  ros::NodeHandle nh{ "~" };
  nh.setParam(JOINT_WITH_UNDEFINED_HAS_ACC_LIM, true);

  EXPECT_THROW(runGetJointAccelerationLimits(nh, { JOINT_WITH_UNDEFINED_HAS_ACC_LIM }), ros::InvalidParameterException);
}

}  // namespace pilz_joint_trajectory_controller_test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_get_joint_acceleration_limits");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
