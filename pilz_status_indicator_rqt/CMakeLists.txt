cmake_minimum_required(VERSION 2.8.3)
project(pilz_status_indicator_rqt)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)
catkin_package()
catkin_python_setup()

install(PROGRAMS scripts/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(ros_pytest REQUIRED)

  file(GLOB unittest_scripts "test/unit_tests/*.py")

  if(ENABLE_COVERAGE_TESTING)
    foreach(file ${unittest_scripts})
      add_pytests(${file} OPTIONS "--cov --cov-append")
    endforeach()
  else()
    foreach(file ${unittest_scripts})
      add_pytests(${file})
    endforeach()
  endif()

  # needs https://github.com/machinekoder/ros_pytest/pull/3
  #if(ENABLE_COVERAGE_TESTING)
  #  add_pytests(test/unit_tests OPTIONS "--cov --cov-append")
  #else()
  #  add_pytests(test/unit_tests)
  #endif()

  # run: catkin_make -DENABLE_COVERAGE_TESTING=ON pilz_status_indicator_rqt_coverage -j1
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      DEPENDENCIES tests
    )
  endif()
endif()