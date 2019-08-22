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

namespace prbt_hardware_support
{

/**
 * @brief Tests that exception is thrown if empty callback function is
 * specified.
 */
TEST(FilterPipelineTest, testEmptyCallbackFunction){
  {
    ros::NodeHandle nh;
    FilterPipeline::TCallbackFunc cb;
    EXPECT_THROW(FilterPipeline(nh, cb), std::invalid_argument);
  }
}

} // namespace prbt_hardware_support

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_filter_pipeline");
  ros::NodeHandle nh;  // This nodehandle is held, to avoid rosconsole::shutdown(), which results in subsequent ROS_* messages no longer output.
  while (!ros::ok())
  {
    ROS_INFO("waiting for ros to be ok ...");
    ros::Duration(0.1).sleep(); // making sure ros is ok
  }

  testing::InitGoogleTest(&argc, argv);

  int res = RUN_ALL_TESTS();
  ros::Duration(0.1).sleep(); // making sure all log output is received
  nh.shutdown();
  return res;
}
