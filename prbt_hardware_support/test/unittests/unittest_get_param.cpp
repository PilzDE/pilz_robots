/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#include <string>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <prbt_hardware_support/get_param.h>

namespace prbt_hardware_support
{

static const std::string DUMMY_PARAM_NAME{"dummy_parameter"};
static const std::string FALSE_PARAM_NAME{"false_name"};
static const std::string DUMMY_PARAM_VALUE{"parameter value"};

/**
 * Test description.
 *
 * Test Sequence:
 *  1. Call getParam with correct parameter name and type.
 *  2. Call getParam with correct parameter name but wrong type.
 *  3. Call getParam with wrong parameter name but correct type.
 *
 * Expected Results:
 *  1. No exception is thrown and the the expected parameter value is returned.
 *  2. getParam throws an exception.
 *  3. getParam throws an exception.
 */
TEST(GetParamTest, testAll)
{
  std::string param_value;
  ros::NodeHandle nh{"~"};

  // STEP 1
  try
  {
    param_value = getParam<std::string>(nh, DUMMY_PARAM_NAME);
    EXPECT_EQ(DUMMY_PARAM_VALUE, param_value) << "Unexpected pararmeter value read from parameter server.";
  }
  catch(const GetParamException& e)
  {
    ADD_FAILURE() << e.what();
  }

  // STEP 2
  EXPECT_THROW(getParam<int>(nh, DUMMY_PARAM_NAME), GetParamException);

  // STEP 3
  EXPECT_THROW(getParam<std::string>(nh, FALSE_PARAM_NAME), GetParamException);
}

} // namespace prbt_hardware_support

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_get_param");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
