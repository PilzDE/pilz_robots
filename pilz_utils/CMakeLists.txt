cmake_minimum_required(VERSION 2.8.3)
project(pilz_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
)

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

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif()

  # --- getParam unit test ---
  add_rostest_gtest(unittest_get_param
    test/unittest_get_param.test
    test/unittest_get_param.cpp
  )
  target_link_libraries(unittest_get_param
    ${catkin_LIBRARIES}
  )
  #----------------------------------

  # --- waitForService unit test ---
  add_rostest_gtest(unittest_wait_for_service
     test/unittest_wait_for_service.test
     test/unittest_wait_for_service.cpp
  )
  target_link_libraries(unittest_wait_for_service
    ${catkin_LIBRARIES}
  )
  #----------------------------------

  # --- waitForMessage unit test ---
  add_rostest_gtest(unittest_wait_for_message
     test/unittest_wait_for_message.test
     test/unittest_wait_for_message.cpp
  )
  target_link_libraries(unittest_wait_for_message
    ${catkin_LIBRARIES}
  )
  #----------------------------------

  # to run: catkin_make -DENABLE_COVERAGE_TESTIN=ON package_name_coverage (adding -j1 recommended)
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      # specifying dependencies in a reliable way is on open issue
      # see https://github.com/mikeferguson/code_coverage/pull/14
      #DEPENDENCIES tests
    )
  endif()
endif()
