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
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

namespace prbt_gazebo
{

class GazeboTest : public testing::Test
{
protected:
  void SetUp() override;
  ros::AsyncSpinner spinner_ {1};
  const ros::Duration WAIT_FOR_ACTION_SERVER_TIME {5};
};

void GazeboTest::SetUp()
{
  spinner_.start();
}

TEST_F(GazeboTest, basicMove)
{
  ros::NodeHandle nh;
  const std::string action_server_name = nh.getNamespace() + "/manipulator_joint_trajectory_controller/follow_joint_trajectory";

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client(action_server_name);
  ASSERT_TRUE(action_client.waitForServer(WAIT_FOR_ACTION_SERVER_TIME));

  // Construct the goal
  control_msgs::FollowJointTrajectoryGoal goal;
  trajectory_msgs::JointTrajectory traj;
  trajectory_msgs::JointTrajectoryPoint traj_point;
  traj_point.positions = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  traj_point.time_from_start = ros::Duration(1);
  traj.points.push_back(traj_point);
  traj.joint_names = {"prbt_joint_1", "prbt_joint_2", "prbt_joint_3", "prbt_joint_4", "prbt_joint_5", "prbt_joint_6"};
  goal.trajectory = traj;

  ASSERT_EQ(actionlib::SimpleClientGoalState::SUCCEEDED, action_client.sendGoalAndWait(goal).state_);
}

}  // namespace prbt_gazebo


int main(int argc, char **argv)
{
  ros::init(argc, argv, "integrationtest_gazebo_bringup");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
