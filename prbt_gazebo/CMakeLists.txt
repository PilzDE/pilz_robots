cmake_minimum_required(VERSION 2.8.3)
project(prbt_gazebo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS actionlib roscpp trajectory_msgs)

catkin_package()

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  include_directories(${catkin_INCLUDE_DIRS})

  add_rostest_gtest(integrationtest_gazebo_bringup
                    test/integrationtest_gazebo_bringup.test
                    test/integrationtest_gazebo_bringup.cpp)
  target_link_libraries(integrationtest_gazebo_bringup ${catkin_LIBRARIES})

endif()
