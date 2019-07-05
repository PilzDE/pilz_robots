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

#include <stdexcept>

#include <gtest/gtest.h>

#include <prbt_hardware_support/filter_pipeline.h>

namespace filter_pipeline_test
{

using namespace prbt_hardware_support;

class FilterPipelineTest : public testing::Test
{

};

/**
 * @brief Tests that exception is thrown if empty callback function is
 * specified.
 */
TEST_F(FilterPipelineTest, testEmptyCallbackFunction)
{
  ros::NodeHandle nh;
  FilterPipeline::TCallbackFunc cb;
  EXPECT_THROW(FilterPipeline(nh, cb), std::invalid_argument);
}


} // namespace filter_pipeline_test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_filter_pipeline");
  ros::NodeHandle nh_;

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
