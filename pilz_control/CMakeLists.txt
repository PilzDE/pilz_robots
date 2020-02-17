cmake_minimum_required(VERSION 3.0.2)
project(pilz_control)

find_package(catkin
  REQUIRED COMPONENTS
    cmake_modules
    roscpp
    std_srvs
    joint_trajectory_controller
    roslint
    controller_manager
    controller_interface
    moveit_core
    moveit_ros_planning
    pilz_msgs
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_srvs
  pilz_msgs
  joint_trajectory_controller
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

################
## Clang tidy ##
################
if(CATKIN_ENABLE_CLANG_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
    )
  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found.")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
  endif()
endif()

add_library(${PROJECT_NAME}
            include/${PROJECT_NAME}/pilz_joint_trajectory_controller.h
            src/pilz_joint_trajectory_controller.cpp
            src/cartesian_speed_monitor.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# install
install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(FILES ${PROJECT_NAME}_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# test
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(pilz_utils REQUIRED)
  find_package(pilz_testutils REQUIRED)

  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif()

  include_directories(test_utils/include)
  include_directories(${pilz_utils_INCLUDE_DIRS})
  include_directories(${pilz_testutils_INCLUDE_DIRS})

  # JOINT_STATES_SPEED_OBSERVER
  add_executable(joint_states_speed_observer
    test_utils/src/joint_states_speed_observer.cpp
  )
  target_link_libraries(joint_states_speed_observer ${catkin_LIBRARIES})

  add_executable(robot_mock
    test/robot_mock_node.cpp
    test/robot_mock.cpp
  )
  target_link_libraries(robot_mock ${catkin_LIBRARIES})

  add_rostest_gmock(unittest_joint_states_speed_observer
    test/unittest_joint_states_speed_observer.test
    test/unittest_joint_states_speed_observer.cpp
  )
  target_link_libraries(unittest_joint_states_speed_observer
    ${catkin_LIBRARIES} ${pilz_testutils_LIBRARIES}
  )
  add_dependencies(unittest_joint_states_speed_observer
    joint_states_speed_observer
    ${catkin_EXPORTED_TARGETS}
  )

  catkin_add_gtest(unittest_cartesian_speed_monitor
    test/unittest_cartesian_speed_monitor.cpp
    src/cartesian_speed_monitor.cpp
  )
  target_link_libraries(unittest_cartesian_speed_monitor
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_traj_mode_state_machine
    test/unittest_traj_mode_state_machine.cpp
  )
  target_link_libraries(unittest_traj_mode_state_machine
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_hold_mode_listener
    test/unittest_hold_mode_listener.cpp
  )
  target_link_libraries(unittest_hold_mode_listener
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_traj_mode_manager
    test/unittest_traj_mode_manager.cpp
  )
  target_link_libraries(unittest_traj_mode_manager
    ${catkin_LIBRARIES}
  )

  add_rostest_gmock(unittest_pilz_joint_trajectory_controller
    test/unittest_pilz_joint_trajectory_controller.test
    test/unittest_pilz_joint_trajectory_controller.cpp
    test/robot_mock.cpp
    src/cartesian_speed_monitor.cpp
  )
  target_link_libraries(unittest_pilz_joint_trajectory_controller ${catkin_LIBRARIES})

  add_rostest_gtest(unittest_pilz_joint_trajectory_controller_is_executing
    test/unittest_pilz_joint_trajectory_controller_is_executing.test
    test/unittest_pilz_joint_trajectory_controller_is_executing.cpp
    test/robot_mock.cpp
    src/cartesian_speed_monitor.cpp
  )
  target_link_libraries(unittest_pilz_joint_trajectory_controller_is_executing ${catkin_LIBRARIES})

  add_rostest_gtest(unittest_get_joint_acceleration_limits
    test/unittest_get_joint_acceleration_limits.test
    test/unittest_get_joint_acceleration_limits.cpp
  )
  target_link_libraries(unittest_get_joint_acceleration_limits ${catkin_LIBRARIES})

  add_rostest(test/integrationtest_pilz_joint_trajectory_controller.test
    DEPENDENCIES ${PROJECT_NAME} robot_mock
  )

  roslint_python(
    test/acceptance_test_speed_monitoring.py
    test/acceptance_test_acceleration_limit.py
    test/integrationtest_pilz_joint_trajectory_controller.py
    test/controller_state_observer.py
    test/holding_mode_service_wrapper.py
    test/trajectory_dispatcher.py
  )
  roslint_add_test()

  # run: catkin_make -DENABLE_COVERAGE_TESTING=ON package_name_coverage
  if(ENABLE_COVERAGE_TESTING)
    APPEND_COVERAGE_COMPILER_FLAGS()
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      DEPENDS tests
    )
  endif()


endif()
