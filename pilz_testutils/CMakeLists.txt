cmake_minimum_required(VERSION 2.8.3)
project(pilz_testutils)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wno-unused-parameter)
add_definitions(-Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pilz_utils
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
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

##########################
## Function definition ##
#########################

function(_catkin_add_library_gmock target)
  if(NOT $GMOCK_FOUND AND NOT GMOCK_FROM_SOURCE_FOUND)
    message(WARNING "skipping GMOCK '${target}' in project '${PROJECT_NAME}' because GMOCK was not found")
    return()
  endif()

  if(NOT DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    message(FATAL_ERROR "_catkin_add_library_gmock() must be called after catkin_package() so that default output directories for the libraries are defined")
  endif()

  cmake_parse_arguments(ARG "EXCLUDE_FROM_ALL" "" "" ${ARGN})

  list(APPEND GMOCK_INCLUDE_DIRS ${GTEST_INCLUDE_DIRS})
  list(APPEND GMOCK_LIBRARY_DIRS ${GTEST_LIBRARY_DIRS})
  list(APPEND GMOCK_LIBRARIES ${GTEST_LIBRARIES})

  # create the executable, with basic + gtest/gmock build flags
  include_directories(${GMOCK_INCLUDE_DIRS})
  link_directories(${GMOCK_LIBRARY_DIRS})
  add_library(${target} ${ARG_UNPARSED_ARGUMENTS})
  if(ARG_EXCLUDE_FROM_ALL)
    set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL TRUE)
  endif()

  assert(GMOCK_LIBRARIES)
  target_link_libraries(${target} ${GMOCK_LIBRARIES})
endfunction()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
_catkin_add_library_gmock(${PROJECT_NAME}
  src/async_test.cpp
  src/joint_state_publisher_mock.cpp
)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
