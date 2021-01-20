cmake_minimum_required(VERSION 3.0.2)
project(prbt_gazebo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS actionlib roscpp trajectory_msgs)

catkin_package()

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

#############
## Install ##
#############

## Install launch and config files
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  include_directories(${catkin_INCLUDE_DIRS})

  add_rostest_gtest(integrationtest_gazebo_bringup
                    test/integrationtest_gazebo_bringup.test
                    test/integrationtest_gazebo_bringup.cpp)
  target_link_libraries(integrationtest_gazebo_bringup ${catkin_LIBRARIES})

endif()
