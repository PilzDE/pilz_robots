cmake_minimum_required(VERSION 2.8.3)
project(prbt_support)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS
)

#############
## Install ##
#############

## Install URDF and meshes
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  find_package(moveit_core REQUIRED)
  find_package(moveit_kinematics REQUIRED)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen3 REQUIRED)

  include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )

  add_rostest_gtest(urdf_tests
  test/urdf_tests.test
  test/urdf_tests.cpp
  )

  target_link_libraries(urdf_tests
    ${catkin_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
    ${Eigen3_LIBRARIES})

endif()

