cmake_minimum_required(VERSION 3.0.2)
project(prbt_moveit_config)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Wno-unused-variable)
add_compile_options(-Werror)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(moveit_core REQUIRED)
  find_package(moveit_ros_planning REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(roscpp REQUIRED)
  find_package(rostest REQUIRED)

  include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
  include_directories(${roscpp_INCLUDE_DIRS})

  if (CATKIN_ENABLE_CLANG_TIDY)
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

  add_rostest_gtest(integrationtest_prbt_ikfast_manipulator_plugin
    test/integrationtest_prbt_ikfast_manipulator_plugin.test
    test/integrationtest_prbt_ikfast_manipulator_plugin.cpp
  )
  target_link_libraries(integrationtest_prbt_ikfast_manipulator_plugin
    ${catkin_LIBRARIES}
    ${moveit_ros_planning_LIBRARIES}
  )

  roslaunch_add_file_check(launch/moveit_planning_execution.launch)
endif()
