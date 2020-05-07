#include <ros/ros.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

typedef std::vector<double> Vec1D;
typedef std::vector<Vec1D> Vec2D;
typedef pluginlib::ClassLoader<kinematics::KinematicsBase> KinematicsLoader;

static const std::string MOVEIT_CORE_PACKAGE {"moveit_core"};
static const std::string KINEMATICS_BASE_CLASS {"kinematics::KinematicsBase"};

static const std::string PLUGIN_NAME_PARAM {"ik_plugin_name"};
static const std::string GROUP_PARAM {"group"};
static const std::string TIP_LINK_PARAM {"tip_link"};
static const std::string ROOT_LINK_PARAM {"root_link"};
static const std::string JOINT_NAMES_PARAM {"joint_names"};
static const std::string ROBOT_DESCRIPTION_PARAM {"robot_description"};

static constexpr double DEFAULT_SEARCH_DISCRETIZATION {0.01};
static constexpr unsigned int NUM_JOINTS {6};
static constexpr double IKFAST_TOLERANCE {0.001};

static constexpr double L1 {0.3500}; // Height of first connector
static constexpr double L2 {0.3070}; // Height of second connector

/**
 * @brief Overload ostream operator for double vectors.
 */
std::ostream& operator<< (std::ostream &os, const Vec1D &vec)
{
  os << "(";
  if (!vec.empty())
  {
    auto it = vec.begin();
    os << *it;
    for (++it; it < vec.end(); ++it)
    {
      os << ", " << *it;
    }
  }
  os << ")";
  return os;
}

/**
 * @brief Overload ostream operator for robot poses.
 */
std::ostream& operator<< (std::ostream &os, const geometry_msgs::Pose &pose)
{
  os << "position: x=" << double(pose.position.x)
     << ", y=" << double(pose.position.y)
     << ", z=" << double(pose.position.z)
     << ", orientation: x=" << double(pose.orientation.x)
     << ", y=" << double(pose.orientation.y)
     << ", z=" << double(pose.orientation.z)
     << ", w=" << double(pose.orientation.w);
  return os;
}

/**
 * @brief Computes joint2 from joint1 such that singular position with px,py=0 is reached.
 */
double singularPoseJointFormula(double joint1)
{
  return ( asin( L1 / L2 * sin(joint1) ) + joint1 );
}

/**
 * @brief Initializes the test of the IKFast plugin.
 */
class PrbtIKFastPluginTest : public ::testing::Test
{
protected:
  //! @brief Loads and initializes the IKFast plugin.
  void SetUp() override;

  //! @brief Ensures that the solver is destroyed before the ClassLoader.
  void TearDown() override;

protected:
  std::string root_link_;
  std::string tip_link_;
  std::string group_name_;
  std::vector<std::string> joint_names_;

  boost::shared_ptr<kinematics::KinematicsBase> solver_;

private:
  ros::NodeHandle ph_ {"~"};
  KinematicsLoader loader_ {MOVEIT_CORE_PACKAGE, KINEMATICS_BASE_CLASS};
};

void PrbtIKFastPluginTest::SetUp()
{
  // load plugin
  std::string plugin_name;
  EXPECT_TRUE(ph_.getParam(PLUGIN_NAME_PARAM, plugin_name))
      << "The parameter " << PLUGIN_NAME_PARAM << " was not found.";

  try
  {
    solver_ = loader_.createInstance(plugin_name);
  }
  catch (pluginlib::PluginlibException &e)
  {
    FAIL() << "Failed to load plugin: " << e.what();
  }

  // initialize plugin
  EXPECT_TRUE(ph_.getParam(GROUP_PARAM, group_name_))
      << "The parameter " << GROUP_PARAM << " was not found.";
  EXPECT_TRUE(ph_.getParam(TIP_LINK_PARAM, tip_link_))
      << "The parameter " << TIP_LINK_PARAM << " was not found.";
  EXPECT_TRUE(ph_.getParam(ROOT_LINK_PARAM, root_link_))
      << "The parameter " << ROOT_LINK_PARAM << " was not round.";
  EXPECT_TRUE(ph_.getParam(JOINT_NAMES_PARAM, joint_names_))
      << "The parameter " << JOINT_NAMES_PARAM << " was not found.";

  std::vector<std::string> tip_links {tip_link_};
  robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION_PARAM, false);
  const robot_model::RobotModelPtr& robot_model = robot_model_loader.getModel();

  ASSERT_TRUE( solver_->initialize(*robot_model, group_name_, root_link_, tip_links,
                                   DEFAULT_SEARCH_DISCRETIZATION) ) << "Failed to initialize plugin.";
}

void PrbtIKFastPluginTest::TearDown()
{
  solver_.reset();
}

/**
 * @brief Tests if the IKFast plugin can solve the IK problem for specific singular poses.
 *
 * Test Sequence:
 *    1. Define a set of test joint angles. Perform the following steps for all data.
 *    2. Compute pose via forward kinematics.
 *    3. Solve IK problem.
 *    4. Compute deviation of solutions to the test joint angles.
 *
 * Expected Results:
 *    1. -
 *    2. One pose is obtained.
 *    3. At least one solution is obtained.
 *    4. At least one solution is close to the test joint angles.
 */
TEST_F(PrbtIKFastPluginTest, testSingularities)
{
  // ++++++++++
  // + Step 1 +
  // ++++++++++

  std::vector<std::string> link_names { {solver_->getTipFrame()} };
  Vec1D seed;
  seed.resize(NUM_JOINTS, 0.);

  Vec2D joints_test_set;
  joints_test_set.push_back(Vec1D { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} });
  joints_test_set.push_back(Vec1D { {0.0, 0.0, 0.0, 1.0, 0.0, 0.0} });
  joints_test_set.push_back(Vec1D { {0.0, 0.0, 0.0, 0.0, -1.0, 0.0} });
  joints_test_set.push_back(Vec1D { {0.0, 0.0, 0.0, 0.0, 1.0, 1.0} });

  double j1 { 1.0 };
  double j2 { singularPoseJointFormula(j1) };
  joints_test_set.push_back(Vec1D { {0.0, j1, j2, 0.0, 0.0, 0.0} });
  joints_test_set.push_back(Vec1D { {0.0, j1, j2, -1.0, 0.0, 0.0} });
  joints_test_set.push_back(Vec1D { {0.0, j1, j2, 0.0, 1.0, 0.0} });
  joints_test_set.push_back(Vec1D { {0.0, j1, j2, 0.0, 1.0, -1.0} });

  // Same procedure for all joint vectors
  for (Vec1D joints : joints_test_set)
  {
    ROS_INFO_STREAM("Using joint angles: " << joints);

    // ++++++++++
    // + Step 2 +
    // ++++++++++

    std::vector<geometry_msgs::Pose> poses;
    ASSERT_TRUE( solver_->getPositionFK(link_names, joints, poses) )
        << "Failed to compute forward kinematics.";
    EXPECT_EQ(1u, poses.size());
    ROS_INFO_STREAM("Obtain pose: " << poses.at(0));

    // ++++++++++
    // + Step 3 +
    // ++++++++++

    Vec2D solutions;
    kinematics::KinematicsResult result;
    kinematics::KinematicsQueryOptions options;

    ros::Time generation_begin = ros::Time::now();
    EXPECT_TRUE( solver_->getPositionIK(poses, seed, solutions, result, options) )
        << "Failed to solve inverse kinematics.";
    double duration_ms = (ros::Time::now() - generation_begin).toSec() * 1000;
    ROS_DEBUG_STREAM("Ik solve took " << duration_ms << " ms");


    ROS_INFO_STREAM("Received " << solutions.size() << " solutions to IK problem.");
    EXPECT_GT(solutions.size(), 0u);

    // ++++++++++
    // + Step 4 +
    // ++++++++++

    bool found_expected_solution {false};
    for (Vec1D sol : solutions)
    {
      EXPECT_EQ(sol.size(), NUM_JOINTS);

      double diff {0.0};
      for (unsigned int j = 0; j < NUM_JOINTS; j++)
      {
        diff += pow(sol[j] - joints[j], 2);
      }

      found_expected_solution = (sqrt(diff) < IKFAST_TOLERANCE);
      if (found_expected_solution)
      {
        break;
      }
    }

    if (!found_expected_solution)
    {
      ADD_FAILURE() << "No solution is near the expected values.";
      // print all solutions in case of failure
      for (const Vec1D& sol : solutions)
      {
        ROS_INFO_STREAM("Solution: " << sol);
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_prbt_ikfast_manipulator_plugin");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
