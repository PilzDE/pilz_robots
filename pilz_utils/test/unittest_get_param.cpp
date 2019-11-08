/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
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

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <pilz_utils/get_param.h>

namespace pilz_utils
{

static const std::string DUMMY_PARAM_NAME{"dummy_parameter"};
static const std::string FALSE_PARAM_NAME{"false_name"};
static const std::string DUMMY_PARAM_VALUE{"parameter value"};

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST(GetParamTest, testGetParamExceptionDtor)
{
  {
    std::shared_ptr<GetParamException> ex {new GetParamException("Test msg")};
  }
}

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

} // namespace pilz_utils

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_get_param");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
