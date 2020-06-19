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

#include <cmath>
#include <math.h>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/Pose.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pilz_control/cartesian_speed_monitor.h>
#include <pilz_control/cartesian_speed_monitor_exception.h>

static constexpr double SPEED_COMPARISON_TOLERANCE{ 0.001 };
static constexpr double SMALL_SKIP{ 0.01 };

static void buildTestModel(moveit::core::RobotModelConstPtr& model)
{
  moveit::core::RobotModelBuilder builder("test_robot", "base");

  // keep in mind that we always rotate around the x-axis, so
  // by the following transformation to the origin of link1 we obtain
  // a rotation of link1 around the (base) y-axis
  geometry_msgs::Pose origin1;
  tf2::Quaternion quat1;
  quat1.setRPY(0.0, 0.0, -0.5 * M_PI);
  origin1.orientation = tf2::toMsg(quat1);

  // link2 is rotated about the z-axis tilted about 45 degree in the x-z-plane
  // the default position is perpendicular to link1 and parallel to the rotation axis of joint1
  geometry_msgs::Pose origin2;
  origin2.position.z = 1.0;
  tf2::Quaternion quat2;
  quat2.setRPY(0.0, -0.25 * M_PI, 0.5 * M_PI);
  origin2.orientation = tf2::toMsg(quat2);

  builder.addChain("base->link1->link2", "continuous", { origin1, origin2 });

  geometry_msgs::Pose origin3;
  origin3.position.y = 1.0;

  // as a result if we fix joint2 such that link2 is in perpendicular position and move joint1,
  // the maximal velocity is in the origins of link2 and link3 which move equally fast
  // if we fix joint2 such that the angle between link1 and link2 is above 90 degree and move joint1,
  // the origin of link3 moves faster than the origin of link2
  // if we fix joint2 such that the angle between link1 and link2 is below 90 degree and move joint1,
  // the origin of link3 moves slower than the origin of link2
  builder.addChain("link2->link3", "fixed", { origin3 });

  model = builder.build();
}

using pilz_control::CartesianSpeedMonitor;

class CartesianSpeedMonitorTest : public testing::Test
{
protected:
  void SetUp() override;

protected:
  moveit::core::RobotModelConstPtr model_;
  std::vector<std::string> joint_names_;
};

void CartesianSpeedMonitorTest::SetUp()
{
  buildTestModel(model_);
  joint_names_ = model_->getVariableNames();
}

TEST_F(CartesianSpeedMonitorTest, testUnmatchedJointNames)
{
  joint_names_.push_back("invalid_joint_name");
  EXPECT_THROW(CartesianSpeedMonitor monitor(joint_names_, model_), pilz_control::RobotModelVariableNamesMismatch);
}

/**
 * @tests{Monitor_speed_of_all_links_until_TCP,
 * Tests correct link speed for no motion.
 * }
 */
TEST_F(CartesianSpeedMonitorTest, testLinkSpeedNoMovement)
{
  robot_state::RobotStatePtr state(new robot_state::RobotState(model_));
  state->setToDefaultValues();
  state->updateLinkTransforms();
  const moveit::core::LinkModel* link = model_->getLinkModel("link2");
  const double time_delta = 1.0;

  double speed = pilz_control::linkSpeed(state.get(), state.get(), link, time_delta);
  EXPECT_NEAR(speed, 0.0, SPEED_COMPARISON_TOLERANCE);

  link = model_->getLinkModel("link3");
  speed = pilz_control::linkSpeed(state.get(), state.get(), link, time_delta);
  EXPECT_NEAR(speed, 0.0, SPEED_COMPARISON_TOLERANCE);
}

/**
 * @tests{Monitor_speed_of_all_links_until_TCP,
 * Tests correct link speed moving a single joint.
 * }
 */
TEST_F(CartesianSpeedMonitorTest, testLinkSpeedMoveSingleJoint)
{
  robot_state::RobotStatePtr state(new robot_state::RobotState(model_));
  state->setToDefaultValues();
  state->updateLinkTransforms();

  robot_state::RobotStatePtr state2(new robot_state::RobotState(model_));
  const double angular_displacement{ 0.1 };
  const double time_delta{ 0.3 };

  for (const std::string& joint_name : joint_names_)
  {
    state2->setToDefaultValues();
    state2->setVariablePosition(joint_name, angular_displacement);
    state2->updateLinkTransforms();

    const moveit::core::LinkModel* link = model_->getLinkModel("link2");
    double speed = pilz_control::linkSpeed(state.get(), state2.get(), link, time_delta);
    double expected_velocity = (joint_name == "base-link1-joint") ? (angular_displacement / time_delta) : 0.0;
    EXPECT_NEAR(speed, expected_velocity, SPEED_COMPARISON_TOLERANCE);

    link = model_->getLinkModel("link3");
    speed = pilz_control::linkSpeed(state.get(), state2.get(), link, time_delta);
    expected_velocity = angular_displacement / time_delta;
    EXPECT_NEAR(speed, expected_velocity, SPEED_COMPARISON_TOLERANCE);
  }
}

/**
 * @tests{Monitor_speed_of_all_links_until_TCP,
 * Tests that link speed scales linearly in time.
 * }
 */
TEST_F(CartesianSpeedMonitorTest, testLinkSpeedLinearScalingInTime)
{
  robot_state::RobotStatePtr state(new robot_state::RobotState(model_));
  state->setToDefaultValues();
  state->updateLinkTransforms();

  robot_state::RobotStatePtr state2(new robot_state::RobotState(model_));
  state2->setToDefaultValues();
  std::vector<double> angular_displacements{ 0.1, 0.1 };
  state2->setVariablePositions(angular_displacements);
  state2->updateLinkTransforms();

  const moveit::core::LinkModel* link = model_->getLinkModel("link3");
  double time_delta = 0.1;

  const double speed = pilz_control::linkSpeed(state.get(), state2.get(), link, time_delta);

  const double factor = 2.0;
  time_delta *= factor;
  const double speed2 = pilz_control::linkSpeed(state.get(), state2.get(), link, time_delta);

  EXPECT_NEAR(speed2 * factor, speed, SPEED_COMPARISON_TOLERANCE);
}

/**
 * @tests{Monitor_speed_of_all_links_until_TCP,
 * Tests monitoring all links moving a single joint.
 * }
 */
TEST_F(CartesianSpeedMonitorTest, testBelowLimitMoveSingleJoint)
{
  CartesianSpeedMonitor monitor(joint_names_, model_);
  monitor.init();

  const double angular_displacement = 0.1;
  const double time_delta = 0.2;

  const std::vector<double> current_positions(joint_names_.size(), 0.0);

  for (unsigned int joint_index = 0; joint_index < joint_names_.size(); ++joint_index)
  {
    std::vector<double> desired_positions(joint_names_.size(), 0.0);
    desired_positions.at(joint_index) = angular_displacement;

    double limit = angular_displacement / time_delta + SMALL_SKIP;
    EXPECT_TRUE(monitor.cartesianSpeedIsBelowLimit(current_positions, desired_positions, time_delta, limit));

    limit = angular_displacement / time_delta - SMALL_SKIP;
    EXPECT_FALSE(monitor.cartesianSpeedIsBelowLimit(current_positions, desired_positions, time_delta, limit));
  }
}

/**
 * @tests{Monitor_speed_of_all_links_until_TCP,
 * Tests monitoring all links moving a single joint when the links are not in their default perpendicular position.
 * }
 */
TEST_F(CartesianSpeedMonitorTest, testBelowLimitLinksNotPerpendicular)
{
  CartesianSpeedMonitor monitor(joint_names_, model_);
  monitor.init();

  const double angular_displacement = 0.1;
  const double time_delta = 0.2;

  // put link2/3 out of perpendicular position
  const double limit = angular_displacement / time_delta + SMALL_SKIP;
  EXPECT_FALSE(
      monitor.cartesianSpeedIsBelowLimit({ 0.0, 0.5 * M_PI }, { angular_displacement, 0.5 * M_PI }, time_delta, limit));
}

TEST_F(CartesianSpeedMonitorTest, testD0Destructor)
{
  std::shared_ptr<CartesianSpeedMonitor> monitor{ new CartesianSpeedMonitor(joint_names_, model_) };
}

TEST_F(CartesianSpeedMonitorTest, testD0DestructorException)
{
  using pilz_control::RobotModelVariableNamesMismatch;
  std::shared_ptr<RobotModelVariableNamesMismatch> monitor{ new RobotModelVariableNamesMismatch() };
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
