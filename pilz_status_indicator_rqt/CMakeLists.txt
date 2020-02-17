cmake_minimum_required(VERSION 3.0.2)
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

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_nosetests(test/unit_tests/status_indicator_unit_test.py)
  catkin_add_nosetests(test/unit_tests/status_indicator_widget_unit_test.py)

  # To generate coverage run:
  #     python-coverage run -a --source src test/unit_tests/status_indicator_unit_test.py && \
  #     python-coverage run -a --source src test/unit_tests/status_indicator_widget_unit_test.py && \
  #     python-coverage html && see htmlcov/index.html
endif()