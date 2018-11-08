cmake_minimum_required(VERSION 2.8.3)
project(prbt_hardware_support)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wno-unused-parameter)
add_definitions(-Werror)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  message_generation
  message_filters
)

# message generation
add_message_files(
  FILES
  ModbusMsgInStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include test/include
  CATKIN_DEPENDS message_runtime std_msgs
  LIBRARIES ${PROJECT_NAME} prbt_hardware_support
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  test/include
)

add_executable(sto_modbus_adapter_node
  src/sto_modbus_adapter_node.cpp
  src/sto_modbus_adapter.cpp
  src/modbus_msg_sto_wrapper.cpp
)
add_dependencies(sto_modbus_adapter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(sto_modbus_adapter_node ${catkin_LIBRARIES})

# +++++++++++++++++++++++++++++++++
# + Build modbus read client node +
# +++++++++++++++++++++++++++++++++
add_executable(
  pilz_modbus_read_client_node
  src/pilz_modbus_read_client_node.cpp
  src/pilz_modbus_read_client.cpp
  src/libmodbus_client.cpp
)
add_dependencies(pilz_modbus_read_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(pilz_modbus_read_client_node ${catkin_LIBRARIES} modbus)

#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ test/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(TARGETS sto_modbus_adapter_node pilz_modbus_read_client_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(code_coverage REQUIRED)

  add_rostest_gmock(unittest_update_filter
                    test/unittests/unittest_update_filter.test
                    test/unittests/unittest_update_filter.cpp
  )
  target_link_libraries(unittest_update_filter
    ${catkin_LIBRARIES}
  )
  add_dependencies(unittest_update_filter ${${PROJECT_NAME}_EXPORTED_TARGETS})


  catkin_add_gmock(unittest_libmodbus_client
      test/unittests/unittest_libmodbus_client.cpp
      test/unittests/pilz_modbus_server_mock.cpp
      src/libmodbus_client.cpp

  )
  target_link_libraries(unittest_libmodbus_client
    ${catkin_LIBRARIES}
    modbus
  )
  add_dependencies(unittest_libmodbus_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

  catkin_add_gtest(unittest_pilz_modbus_exception
                   test/unittests/unittest_pilz_modbus_exception.cpp)
  add_dependencies(unittest_pilz_modbus_exception ${${PROJECT_NAME}_EXPORTED_TARGETS})


  catkin_add_gtest(unittest_pilz_modbus_read_client_exception
  test/unittests/unittest_pilz_modbus_read_client_exception.cpp)
  add_dependencies(unittest_pilz_modbus_read_client_exception ${${PROJECT_NAME}_EXPORTED_TARGETS})

  #--- PilzModbusReadClient unit test ---
  add_rostest_gmock(unittest_pilz_modbus_read_client
      test/unittests/unittest_pilz_modbus_read_client.test
      test/unittests/unittest_pilz_modbus_read_client.cpp
      src/pilz_modbus_read_client.cpp
  )
  target_link_libraries(unittest_pilz_modbus_read_client
    ${catkin_LIBRARIES}
  )
  add_dependencies(unittest_pilz_modbus_read_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

  #--- StoModbusAdapter unit test ---
  add_rostest_gmock(unittest_sto_modbus_adapter
   test/unittests/unittest_sto_modbus_adapter.test
   test/unittests/unittest_sto_modbus_adapter.cpp
   src/sto_modbus_adapter.cpp
   src/modbus_msg_sto_wrapper.cpp
  )
  target_link_libraries(unittest_sto_modbus_adapter ${catkin_LIBRARIES})
  #----------------------------------

  # --- Stop integration test ---
  add_rostest_gmock(integrationtest_stop1
     test/integrationtests/integrationtest_stop1.test
     test/integrationtests/integrationtest_stop1.cpp
     test/unittests/pilz_modbus_server_mock.cpp
     src/libmodbus_client.cpp
  )
  target_link_libraries(integrationtest_stop1
   ${catkin_LIBRARIES}
   modbus
  )
  add_dependencies(integrationtest_stop1 ${catkin_EXPORTED_TARGETS} pilz_modbus_read_client_node sto_modbus_adapter_node)

  # to run: catkin_make -DENABLE_COVERAGE_TESTING=ON package_name_coverage (adding -j1 recommended)
  if(ENABLE_COVERAGE_TESTING)
    include(CodeCoverage)
    APPEND_COVERAGE_COMPILER_FLAGS()
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "ModbusMsgInStamped.h")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      DEPENDENCIES tests
    )
  endif()
endif()
