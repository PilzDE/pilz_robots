cmake_minimum_required(VERSION 2.8.12)
project(prbt_ikfast_manipulator_plugin)

add_compile_options(-std=c++14)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Wno-unused-variable)
add_compile_options(-Werror)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf2_kdl
  tf2_eigen
  eigen_conversions
)
find_package(LAPACK REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} include)

catkin_package()

set(IKFAST_LIBRARY_NAME prbt_manipulator_moveit_ikfast_plugin)
add_library(${IKFAST_LIBRARY_NAME} src/prbt_manipulator_ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})
# suppress warnings about unused variables in OpenRave's solver code
target_compile_options(${IKFAST_LIBRARY_NAME} PRIVATE -Wno-unused-variable)

install(TARGETS
  ${IKFAST_LIBRARY_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  prbt_manipulator_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# unittest of ikfast plugin
if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  find_package(code_coverage REQUIRED)
  find_package(moveit_ros_planning REQUIRED)

  include_directories(include ${catkin_INCLUDE_DIR})

  add_rostest_gtest(unittest_${PROJECT_NAME}
    test/unittests/tst_${PROJECT_NAME}.test
    test/unittests/tst_${PROJECT_NAME}.cpp
  )

  target_link_libraries(unittest_${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
  )

  # run: catkin_make -DENABLE_COVERAGE_TESTING=ON package_name_coverage
  if(ENABLE_COVERAGE_TESTING)
    include(CodeCoverage)
    APPEND_COVERAGE_COMPILER_FLAGS()
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      DEPENDS tests
    )
  endif()

endif()
